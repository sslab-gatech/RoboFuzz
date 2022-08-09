#!/usr/bin/python3

from collections import defaultdict
import sched

import time
import os
import sys
import traceback
from functools import reduce
import random
import struct
import pickle
import argparse
from datetime import datetime
import asyncio
import math
import subprocess as sp
import signal
import pprint
import importlib
from collections import deque
from copy import deepcopy
import json
import shutil

import numpy as np
import sysv_ipc as ipc

import constants as c
import config
import ros_utils
import mutator
import harness
import checker
from rosbag_parser import RosbagParser
from checker import StateMonitorNode
from executor import Executor, ExecMode
from scheduler import Scheduler, Campaign
from feedback import Feedback, FeedbackType

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from ros2node.api import *

# from ros2msg.api import get_all_message_types, get_message_path
# from ros2srv.api import get_all_service_types
from rqt_graph.rosgraph2_impl import Graph
from ros2topic.api import get_msg_class
from rosidl_runtime_py import message_to_ordereddict, set_message_fields

from ros2_fuzzer import ros_commons
from ros2_fuzzer.process_handling import FuzzedNodeHandler

from std_msgs.msg import Bool, String
from tracer import APITracer
from turtlesim.msg import Pose


class Fuzzer:
    node_ptr = None

    def __init__(self, node_name, config):
        self.config = config
        self.node_name = node_name
        self.node_ptr = rclpy.create_node(node_name)
        self.subs = []
        self.coverage_map = {}
        self.cov_last_update = time.time()
        self.loop = 0
        self.rounds = 0
        self.pub = None
        self.client = None
        self.shm = None
        self.shm_data = None

    def init_cov_map(self):
        print("[*] Initializing shm for coverage tracking")
        if self.config.px4_sitl:
            self.shm = None
            return

        if self.config.sros2:
            self.shm = None
            return

        if self.config.test_moveit:
            self.shm = None
            return

        if self.config.no_cov:
            self.shm = None
            return

        key = ipc.ftok(self.config.proj_root, 2333)
        try:
            self.shm = ipc.SharedMemory(
                key,
                flags=ipc.IPC_CREX,
                size=pow(2, 16),
                init_character=b"\x00",
            )
            print("shmid:", self.shm)
            with open("/tmp/shmid", "w") as fp:
                fp.write(str(self.shm.id))
        except ipc.ExistentialError:
            print("cannot create shm")
            exit(-1)

        if self.config.debug_wait:
            x = input("waiting. press any key")

        self.cov_map = [0] * pow(2, 16)

    def init_shm_data(self):
        print("[*] Initializing shm for data transmission")

        key = ipc.ftok(self.config.proj_root, 2334)

        try:
            self.shm_data = ipc.SharedMemory(
                key,
                flags=ipc.IPC_CREX,
                size=pow(2, 16), # should be bigger?
                init_character=b"\x00",
            )
            print("shm_data id:", self.shm_data, self.shm_data.id)
            self.config.test_rosidl_shmid = self.shm_data.id
            # with open("/tmp/shmid", "w") as fp:
                # fp.write(str(self.shm_data.id))
        except ipc.ExistentialError:
            print("cannot create shm_data")
            exit(-1)

    def init_queue(self):
        print("[*] Initializing test case queue")

        self.queue = deque()

        if self.config.px4_sitl:
            if self.config.fuzz_seed:
                msg_list = px4_utils.read_trajectory_seed(self.config.fuzz_seed)

            elif self.config.exp_pgfuzz:
                # campaign: single
                msg = px4_utils.get_init_parameter_msg()
                msg_list = msg

            else:
                # campaign: sequence
                if self.config.use_mavlink:
                    msg = px4_utils.get_init_manual_control_msg()
                else:
                    msg = px4_utils.get_init_trajectory_msg()

                msg_list = list()
                for i in range(self.config.seqlen):
                    msg_list.append(deepcopy(msg))

            self.queue.append(msg_list)

        elif self.config.test_moveit:
            # when using joint constraints
            # msg = harness.get_init_moveit_msg()
            # joint_constraints = msg.goal_constraints[0].joint_constraints
            # self.queue.append(joint_constraints)

            msg = harness.get_init_moveit_pose()

            # override the default msg with the given seed
            if self.config.fuzz_seed:
                f = open(self.config.fuzz_seed, "rb")
                msg_dict = pickle.load(f)
                msg.position.x = msg_dict["position"]["x"]
                msg.position.y = msg_dict["position"]["y"]
                msg.position.z = msg_dict["position"]["z"]
                msg.orientation.w = msg_dict["orientation"]["w"]

            self.queue.append(msg)

    # def start_virtual_display(self):
    # self.display = Display(visible=1, size=(1280, 720))
    # self.display.start()

    def init_px4_bridge(self):
        print("[*] Target: PX4 SITL")
        print("[*] Initializing PX4-ROS bridge")
        self.px4_bridge = px4_utils.Px4BridgeNode(
            use_mavlink=self.config.use_mavlink
        )

    def init_state_monitor(self, watchlist_file):
        """
        Run state monitor node in the background to keep it spinning
        for listening to the subscribed topics and dumping messages
        """

        monitor_run_cmd = f"python3 state_monitor.py {watchlist_file}"

        self.state_monitor_pgrp = sp.Popen(
            monitor_run_cmd,
            shell=True,
            preexec_fn=os.setpgrp,
            stdout=sp.PIPE,
            stderr=sp.PIPE,
        )

    def run_target(self, ros_pkg, ros_node, exec_cmd):

        # os.system("ros2 daemon start")

        if self.node_ptr is None:
            self.node_ptr = rclpy.create_node("_fuzzer")

        if self.config.px4_sitl:
            print("[*] Starting PX4 SITL stack & Gazebo simulator")
            proc = harness.run_px4_stack_sh(self.config.proj_root)
            time.sleep(10)  # TODO: check gazebo status rather than waiting
            print("[px4] started px4 sitl stack")
            self.running = True
            return

        elif self.config.tb3_sitl:
            print("[*] Starting TurtleBot3 SITL stack & Gazebo simulator")
            self.ros_pgrp = harness.run_tb3_sitl(self.config.proj_root)
            time.sleep(10)
            print("[tb3] started tb3 sitl stack", self.ros_pgrp.pid)
            self.running = True
            return

        elif self.config.tb3_hitl:
            print("[*] Starting TurtleBot3 hardware")
            proc = harness.run_tb3_hitl(self.config.tb3_uri)
            time.sleep(10)
            print("[tb3] started turtlebot3 burger")
            self.running = True
            return

        if self.config.test_rcl:
            print("[*] Starting RCL harness")
            self.ros_pgrp = harness.run_rcl_api_harness(
                self.config.test_rcl_feature,
                self.config.test_rcl_targets,
                self.config.test_rcl_job,
            )

            self.running = True

            # add slight delay for topic discovery
            time.sleep(5)

            return

        if self.config.test_cli:
            print("[*] Starting CLI harness")

            self.ros_pgrp = harness.run_cli_harness()

            self.running = True

            # add slight delay for topic discovery
            time.sleep(5)

            return

        if self.config.test_rosidl:
            print("[*] Starting ROSIDL harness")
            self.ros_pgrp = harness.run_rosidl_harness(
                self.config.test_rosidl_lang,
                0, # self.config.test_rosidl_shmid,
                "empty",
            )

            self.running = True

            time.sleep(1)

            return

        if self.config.test_moveit:
            print("[*] Starting moveit2 harness")
            self.ros_pgrp = harness.run_moveit_harness()

            self.running = True

            time.sleep(15)

            return

        if exec_cmd is not None:
            # Non-ROS testing (e.g., PX4)
            # ros_run_cmd = exec_cmd
            ros_run_cmd = exec_cmd
            sleep_interval = 1
        else:
            # ROS testing
            if os.path.exists(self.config.node_executable):
                ros_run_cmd = self.config.node_executable
                sleep_interval = 0.25
            else:
                ros_run_cmd = "ros2 run {} {}".format(ros_pkg, ros_node)
                sleep_interval = 1

            if self.config.sros2:
                env = ""
                env += f"ROS_SECURITY_KEYSTORE={self.config.sros2_keystore} "
                env += f"ROS_SECURITY_ENABLE={self.config.sros2_enable} "
                env += f"ROS_SECURITY_STRATEGY={self.config.sros2_strategy} "
                ros_run_cmd = env + ros_run_cmd
                ros_run_cmd += (
                    f" --ros-args --enclave {self.config.sros2_enclave}"
                )

        if not os.path.exists(os.path.join(self.config.log_dir, "run_cmd")):
            with open(os.path.join(self.config.log_dir, "run_cmd"), "w") as fp:
                fp.write(ros_run_cmd)

        if self.config.sros2:
            try:
                os.remove("/tmp/sros2_started")
            except:
                pass

        self.ros_pgrp = sp.Popen(
            ros_run_cmd,
            shell=True,
            preexec_fn=os.setpgrp,
            stdout=sp.PIPE,
            stderr=sp.PIPE,
        )

        if self.config.sros2:
            while True:
                if os.path.exists("/tmp/sros2_started"):
                    break
                time.sleep(0.05)
        else:
            time.sleep(sleep_interval)

        print("[ros] started target system")
        self.running = True

    def kill_target(self):
        if not self.running:
            print("[-] nothing to kill")
            return

        if self.config.px4_sitl:
            os.system("pkill px4")
            self.running = False

        elif self.config.tb3_hitl:
            os.system(f"ssh -i keys/tb3 {self.config.tb3_uri} ./kill.sh")
            self.running = False

        else:
            try:
                # send SIGINT instead of SIGTERM so turtle can handle signal
                # and terminate gracefully to create gcda files.
                os.killpg(self.ros_pgrp.pid, signal.SIGKILL)
                self.running = False
            except ProcessLookupError as e:
                print("[-] killpg error:", e)
            except Exception as e:
                print("[-] killpg failed for some reason:", e)

            if self.config.tb3_sitl:
                os.system("pkill gzserver")

        self.node_ptr.destroy_node()
        self.node_ptr = None
        # os.system("ros2 daemon stop")

    def kill_monitor(self):
        # kill state monitor
        try:
            print("killing state monitor")
            os.killpg(self.state_monitor_pgrp.pid, signal.SIGKILL)
        except ProcessLookupError as e:
            print("[-] state monitor killpg error:", e)
        except AttributeError as e:
            print("[-] state monitor not initialized:", e)

    def destroy(self):
        # kill target if still exists
        self.kill_target()
        self.kill_monitor()
        self.executor.kill_rosbag()

        # clear subscriptions
        if self.node_ptr is not None:
            for sub in self.subs:
                self.node_ptr.destroy_subscription(sub)

        try:
            self.display.stop()
        except AttributeError:
            pass

        if self.shm is not None:
            if self.shm.attached:
                self.shm.detach()
            self.shm.remove()

        if self.shm_data is not None:
            if self.shm_data.attached:
                self.shm_data.detach()
            self.shm_data.remove()


def inspect_target(fuzzer):
    fuzz_targets = []

    built_in_msg_types = ros_utils.get_all_message_types()
    subscriptions = ros_utils.get_subscriptions(fuzzer.node_ptr)

    if fuzzer.config.px4_sitl:
        if fuzzer.config.use_mavlink:
            topic_name = "/dummy_mavlink_topic"
            msg_type_class = ros_utils.get_msg_class_from_name(
                "px4_msgs", "ManualControlSetpoint"
            )
        else:
            topic_name = "/TrajectorySetpoint_PubSubTopic"
            msg_type_class = ros_utils.get_msg_class_from_name(
                "px4_msgs", "TrajectorySetpoint"
            )

        fuzz_targets.append([topic_name, msg_type_class, "drone"])
        return fuzz_targets

    elif fuzzer.config.tb3_sitl:
        topic_name = "/cmd_vel"
        msg_type_class = ros_utils.get_msg_class_from_name(
            "geometry_msgs", "Twist"
        )
        fuzz_targets.append(
            [topic_name, msg_type_class, "/turtlebot3_diff_drive"]
        )

        return fuzz_targets

    elif fuzzer.config.tb3_hitl:
        topic_name = "/cmd_vel"
        msg_type_class = ros_utils.get_msg_class_from_name(
            "geometry_msgs", "Twist"
        )
        fuzz_targets.append([topic_name, msg_type_class, "/turtlebot3_node"])

        return fuzz_targets

    elif fuzzer.config.test_rosidl:
        topic_name = "/metatopic" # fake topic (topic tbd by mutator)
        msg_type_class = ros_utils.get_msg_class_from_name(
            "std_msgs", "Empty"
        )
        fuzz_targets.append([topic_name, msg_type_class, "/rosidl_node"])

        return fuzz_targets

    elif fuzzer.config.test_moveit:
        # use below if testing without commander harness
        topic_name = "/motion_plan_request"
        msg_type_class = ros_utils.get_msg_class_from_name(
            "moveit_msgs", "MotionPlanRequest"
        )
        # for now, use commander harness for convenience
        topic_name = "/metatopic"
        msg_type_class = ros_utils.get_msg_class_from_name(
            "geometry_msgs", "Pose"
        )

        fuzz_targets.append([topic_name, msg_type_class, "/moveit_node"])

        return fuzz_targets

    for subscriber_node in subscriptions:
        print()
        print("[+] processing", subscriber_node)

        if subscriber_node.replace("/", "").startswith("_"):
            print("[-] skip internal node")
            continue

        for ti, topic in enumerate(subscriptions[subscriber_node]):
            topic_name = topic[0]
            msg_type_full = topic[1]

            print("[{}] {} {}".format(ti, topic_name, msg_type_full))

            if len(msg_type_full) > 1:
                print("[check] MULTIPLE MESSAGE TYPES!")

            msg_type = msg_type_full[0]
            msg_pkg = msg_type.split("/")[0]
            msg_name = msg_type.split("/")[-1]

            if msg_name == "ParameterEvent":
                print("[-] skip ParameterEvents")
                continue

            if msg_pkg in built_in_msg_types.keys():
                msg_type_class = ros_utils.get_msg_class_from_name(
                    msg_pkg, msg_name
                )
            else:
                msg_type_class = ros_utils.find_custom_msg(msg_type)

            if msg_type_class is None:
                print("[-] couldn't find msg class")
                continue

            print()
            print("Found fuzzing topic", topic_name, "of type", msg_type_class)
            print("- target node:", subscriber_node)
            fuzz_targets.append([topic_name, msg_type_class, subscriber_node])

    return fuzz_targets


def inspect_secure_target(fuzzer):
    fuzz_targets = []

    built_in_msg_types = ros_utils.get_all_message_types()
    subscriptions = ros_utils.get_secure_subscriptions(fuzzer.node_ptr)

    for subscriber_node in subscriptions:
        print()
        print("[+] processing", subscriber_node)

        if subscriber_node.replace("/", "").startswith("_"):
            print("[-] skip internal node")
            continue

        for ti, topic in enumerate(subscriptions[subscriber_node]):
            topic_name = topic[0]
            msg_type_full = topic[1]

            print("[{}] {} {}".format(ti, topic_name, msg_type_full))

            if len(msg_type_full) > 1:
                print("[check] MULTIPLE MESSAGE TYPES!")

            msg_type = msg_type_full[0]
            msg_pkg = msg_type.split("/")[0]
            msg_name = msg_type.split("/")[-1]

            if msg_name == "ParameterEvent":
                print("[-] skip ParameterEvents")
                continue

            if msg_pkg in built_in_msg_types.keys():
                msg_type_class = ros_utils.get_msg_class_from_name(
                    msg_pkg, msg_name
                )
            else:
                msg_type_class = ros_utils.find_custom_msg(msg_type)

            if msg_type_class is None:
                print("[-] couldn't find msg class")
                continue

            print()
            print("Found fuzzing topic", topic_name, "of type", msg_type_class)
            print("- target node:", subscriber_node)
            fuzz_targets.append([topic_name, msg_type_class, subscriber_node])

    fuzzer.kill_target()
    return fuzz_targets


def fuzz_msg(fuzzer, fuzz_targets):

    if len(fuzz_targets) == 0:
        print("[-] Could not discover ROS topic")

    for target in fuzz_targets:
        print("\n----- TARGET INFO -----")
        print("  - TOPIC:", target[0])
        print("  - message type:", target[1])
        print("")

        print("----- BEGIN FUZZING -----")

        # scheduler determines how to mutate, and when to publish mutated messages

        topic_name = target[0]
        msg_type_class = target[1]
        subscriber_node = target[2]

        # campaign = Campaign.RND_SINGLE
        # campaign = Campaign.RND_SEQUENCE
        # campaign = Campaign.RND_REPEATED
        # campaign = Campaign.INTERCEPTION
        campaign = fuzzer.config.schedule
        scheduler = Scheduler(fuzzer, campaign, target)

        field_blacklist = None
        field_whitelist = None
        fbk_list = list()

        # Per-target configuration
        # - whitelist and blacklist
        # - feedback attrs
        if fuzzer.config.px4_sitl:
            # field_blacklist = ["acclelration", "jerk", "thrust"]
            if fuzzer.config.exp_pgfuzz:
                field_whitelist = None

            elif fuzzer.config.use_mavlink:
                field_whitelist = [
                    # ["yawspeed", np.dtype("float64")],
                    ["x", np.dtype("float32")],
                    ["y", np.dtype("float32")],
                    ["z", np.dtype("float32")],
                    ["r", np.dtype("float32")],
                    # ["vx", np.dtype("float64")],
                    # ["vy", np.dtype("float64")],
                    # ["vz", np.dtype("float64")],
                ]

            else:
                field_whitelist = [
                    # ["x", np.dtype("float32")],
                    # ["y", np.dtype("float32")],
                    # ["z", np.dtype("float32")],
                    ["yaw", np.dtype("float64")],
                    ["yawspeed", np.dtype("float64")],
                    ["vx", np.dtype("float64")],
                    ["vy", np.dtype("float64")],
                    ["vz", np.dtype("float64")],
                ]

            fbk = Feedback("imu_accel_inconsistency", FeedbackType.INC)
            fbk_list.append(fbk)
            fbk = Feedback("imu_gyro_inconsistency", FeedbackType.INC)
            fbk_list.append(fbk)

            # gps raw vs estimation
            fbk = Feedback("gps_lat_inconsistency", FeedbackType.INC)
            fbk_list.append(fbk)
            fbk = Feedback("gps_lon_inconsistency", FeedbackType.INC)
            fbk_list.append(fbk)

        elif fuzzer.config.tb3_hitl:
            field_whitelist = [
                ["linear", "x", np.dtype("float64")],
                ["angular", "z", np.dtype("float64")],
            ]

            fbk = Feedback("theta_diff", FeedbackType.INC)
            fbk_list.append(fbk)

        elif fuzzer.config.tb3_sitl:
            field_whitelist = [
                ["angular", "z", np.dtype("float64")],
                ["linear", "x", np.dtype("float64")],
            ]

            fbk = Feedback("theta_diff", FeedbackType.INC)
            fbk_list.append(fbk)

        elif fuzzer.config.rospkg == "turtlesim":
            field_whitelist = [
                ["linear", "x", np.dtype("float64")],
                ["linear", "y", np.dtype("float64")],
                ["angular", "z", np.dtype("float64")]
            ]
            # field_whitelist = None

        elif "turtlebot3_drive" in fuzzer.config.exec_cmd:
            field_whitelist = [
                # ["pose", "pose", "orientation", "x", np.dtype("float64")],
                # ["pose", "pose", "orientation", "y", np.dtype("float64")],
                # ["pose", "pose", "orientation", "z", np.dtype("float64")],
                ["pose", "pose", "orientation", "w", np.dtype("float64")],
            ]

        elif fuzzer.config.test_rosidl:
            # - Gets a metatopic of type std_msgs/Empty
            # - Need to test a moving target (type)
            # - Already know that there's only one field, so the scheduler
            #   can be optimized
            # - Control topic_name, msg_type_class, and subscriber_node
            pass

        elif fuzzer.config.test_moveit:
            # when testing with commander harness
            field_whitelist = [
                ["position", "x", np.dtype("float64")],
                ["position", "y", np.dtype("float64")],
                ["position", "z", np.dtype("float64")],
                # ["orientation", "w", np.dtype("float64")],
            ]

            fbk = Feedback("end_point_deviation", FeedbackType.INC,
                           default_value=0.0)
            fbk_list.append(fbk)

            fbk = Feedback("mean_joint_pos_error", FeedbackType.INC,
                           default_value=0.0)
            fbk_list.append(fbk)

            fbk = Feedback("max_joint_pos_error", FeedbackType.INC,
                           default_value=0.0)
            fbk_list.append(fbk)

            fbk = Feedback("mean_joint_vel_error", FeedbackType.INC,
                           default_value=0.0)
            fbk_list.append(fbk)

            fbk = Feedback("max_joint_vel_error", FeedbackType.INC,
                           default_value=0.0)
            fbk_list.append(fbk)

        scheduler.filter_field_list(field_whitelist, field_blacklist)
        scheduler.init_schedule()

        # fuzzer.pub = fuzzer.node_ptr.create_publisher(msg_type_class, topic_name, 10)
        # fuzzer.state_pub = fuzzer.node_ptr.create_publisher(
        #     Bool,
        #     "_listen_flag",
        #     10
        # )

        executor = Executor(fuzzer)
        fuzzer.executor = executor

        state_monitor = StateMonitorNode(fuzzer)

        if scheduler.campaign == Campaign.RND_SINGLE:
            # mutate and publish one message
            mode = ExecMode.SINGLE
        elif scheduler.campaign == Campaign.RND_SEQUENCE:
            # mutate one from a sequence of messages, publish the sequence
            length = fuzzer.config.seqlen  # todo: apply in scheduler
            mode = ExecMode.SEQUENCE
        elif scheduler.campaign == Campaign.RND_REPEATED:
            # mutate one message and repeatedly publish it for length times
            length = fuzzer.config.seqlen
            mode = ExecMode.SEQUENCE
        elif scheduler.campaign == Campaign.INTERCEPTION:
            mode = ExecMode.SEQUENCE
        elif scheduler.campaign == Campaign.SROS_AUTH:
            mode = ExecMode.SINGLE
        elif scheduler.campaign == Campaign.IDL_CHECK:
            mode = ExecMode.SINGLE

        frequency = 1.0 / config.interval
        repeat = config.repeat

        msg_list = None
        while True:
            errs = []

            if scheduler.campaign == Campaign.RND_SINGLE:
                if fuzzer.config.test_moveit:
                    # only for jointconstraint fuzzing
                    # (mut_msg, frame) = scheduler.mutate_moveit_joint(config)
                    # (mut_msg, frame) = scheduler.mutate_generic(config)
                    (mut_msg, frame) = scheduler.mutate_moveit_goal(config)
                elif fuzzer.config.exp_pgfuzz:
                    # param value mutation like what pgfuzz does
                    (mut_msg, frame) = scheduler.mutate_px4_param(config)

                else:
                    (mut_msg, frame) = scheduler.mutate_generic(config)

                if mut_msg is None:
                    continue

                msg_list = [mut_msg]

            elif scheduler.campaign == Campaign.RND_SEQUENCE:
                if fuzzer.config.use_mavlink:
                    (msg_list, frame) = scheduler.mutate_sequence_mav(config)
                else:
                    (msg_list, frame) = scheduler.mutate_sequence(config)

            elif scheduler.campaign == Campaign.RND_REPEATED:
                (mut_msg, frame) = scheduler.mutate_generic(config)

                if mut_msg is None:
                    continue

                # msg_list = [mut_msg] * random.randint(20, 30)
                msg_list = [mut_msg] * length

            elif scheduler.campaign == Campaign.IDL_CHECK:
                # if (scheduler.round_cnt % 256) == 0:
                (mut_msg, frame, error, expecting) = scheduler.mutate_typemsg(config)
                # print(mut_msg)

                # check for errors that appear early in rosidl_py
                if expecting:
                    if error is None:
                        print(f"{c.RED}[-] Expecting '{expecting}', nothing caught{c.END}")
                        errs.append(f"Expecting '{expecting}', nothing caught")

                        # log early errs
                        if errs:
                            err_file = os.path.join(
                                fuzzer.config.error_dir, f"error-{frame}"
                            )

                            with open(err_file, "a") as fp:
                                fp.write(str(errs))

                    else:
                        print("[+] Expected error caught:")
                        print(error)

                    # error is expected, so don't publish
                    continue

                else:
                    if error:
                        print("{c.RED}[-] Not expecting an error, but caught: {error}{c.END}")
                        errs.append(f"Not expecting an error, caught '{error}'")

                        # log early errs
                        if errs:
                            err_file = os.path.join(
                                fuzzer.config.error_dir, f"error-{frame}"
                            )

                            with open(err_file, "a") as fp:
                                fp.write(str(errs))

                        continue

                msg_list = [mut_msg]

                # publish (via rclpy) to target (rclcpp), and re-publish from
                # target (rclcpp). Whatever msg reached the rclcpp target
                # should not fail while assigning received data to the
                # message. In the meantime, monitor (rosbag) the rclcpp pub
                # topic and check if the messages are identical (checker).

                # XXX: get msg_type_class and topic_name from mut_msg
                # as they're dynamic.
                msg_pkg = "idltest_msgs"
                msg_name = type(mut_msg).__name__

                msg_type_class = ros_utils.get_msg_class_from_name(
                    msg_pkg, msg_name
                )
                topic_name = f"/idltest_{msg_name}_in"

            if msg_list is None:
                continue

            executor.prep_execution(msg_type_class, topic_name)

            # register pre_exec functions and custom publisher function
            if fuzzer.config.px4_sitl:
                collision_checker = checker.CollisionChecker()
                collision_topics = [
                    # "/gazebo/default/iris/base_link/px4_base_contact/contacts",
                    "/gazebo/default/iris/rotor_0/px4_rotor0_contact/contacts",
                    "/gazebo/default/iris/rotor_1/px4_rotor1_contact/contacts",
                    "/gazebo/default/iris/rotor_2/px4_rotor2_contact/contacts",
                    "/gazebo/default/iris/rotor_3/px4_rotor3_contact/contacts",
                ]

                # list of tuple (function, (args))
                pre_exec_list = list()
                if fuzzer.config.use_mavlink:
                    pre_exec_list.append((fuzzer.px4_bridge.init_mavlink, ()))

                if fuzzer.config.exp_pgfuzz:
                    # test px4 by mutating parameter values
                    pre_exec_list.extend([
                        (fuzzer.px4_bridge.prepare_flight, ()),
                        (time.sleep, (5,)),
                        (fuzzer.px4_bridge.mav_set_flight_mode, ("LOITER",)),
                        (time.sleep, (3,)),
                        (collision_checker.listen, (collision_topics,)),
                    ])

                    pub_function = fuzzer.px4_bridge.mav_set_param_msg

                    post_exec_list = [
                        (fuzzer.px4_bridge.mav_revert_param, ()),
                        (collision_checker.stop, ()),
                    ]

                elif fuzzer.config.px4_ros:
                    # test px4 over ROS
                    pre_exec_list.extend([
                        (fuzzer.px4_bridge.prepare_flight, (fuzzer.config.flight_mode,)),
                        (collision_checker.listen, (collision_topics,)),
                    ])

                    pub_function = fuzzer.px4_bridge.send_command

                    post_exec_list = [
                        (collision_checker.stop, ()),
                    ]

                else:
                    # test px4 over mavlink using manual control commands
                    pre_exec_list.extend([
                        (fuzzer.px4_bridge.prepare_flight, (fuzzer.config.flight_mode,)),
                        (fuzzer.px4_bridge.put_in_air, ()),
                        (collision_checker.listen, (collision_topics,)),
                    ])

                    pub_function = fuzzer.px4_bridge.send_command

                    post_exec_list = [
                        (collision_checker.stop, ()),
                    ]

            # ditch the shm stuff
            # elif fuzzer.config.test_rosidl:
                # pub_function = None # writes to the shared memory

            # recover full MotionPlanRequest msg by injecting JointConstraint
            elif fuzzer.config.test_moveit:
                # use below for generic joint constraints
                # # should contain seven (mutated) joint constraints
                # joint_constraints = msg_list[0] # list contains one list
                # print("check")
                # print(joint_constraints)

                # full_msg = harness.get_init_moveit_msg()
                # # goal_constraints is a list of single element
                # goal_constraint = Constraints()
                # goal_constraint.joint_constraints = joint_constraints
                # full_msg.goal_constraints = [goal_constraint]
                # msg_list = [full_msg]

                pre_exec_list = None
                post_exec_list = None
                pub_function = harness.moveit_send_command

            else:
                pre_exec_list = None
                post_exec_list = None
                pub_function = None

            wait_lock = None
            if fuzzer.config.test_rcl or fuzzer.config.test_cli:
                wait_lock = ".waitlock"

            (retval, failure_msg) = executor.execute(
                mode,
                msg_list,
                frame,
                frequency,
                repeat,
                pre_exec_list=pre_exec_list,
                post_exec_list=post_exec_list,
                pub_function=pub_function,
                wait_lock=wait_lock,
            )
            # fuzzer.oh_.check_oracle() # will move everything into checker
            # (turtlesim, sros, ...)
            executor.clear_execution()

            if retval:
                errs.append(f"publish failed: {failure_msg}")

                err_file = os.path.join(
                    fuzzer.config.error_dir, f"error-{frame}"
                )

                with open(err_file, "a") as fp:
                    fp.write(str(errs))

                continue # don't check states as nothing's published

            state_dict_list = []
            # repeated campaigns result in multiple bag files
            for exec_cnt in range(repeat):

                parser = RosbagParser(
                    f"states-{exec_cnt}.bag/states-{exec_cnt}.bag_0.db3"
                )

                if parser.abort:
                    print("[-] corrupted recorded states. Abort.")
                    continue

                state_msgs_dict = parser.process_messages()
                # if dict is empty, fallback to all messages w/o ts filtering
                if len(state_msgs_dict) == 0:
                    print("[-] watch failed")
                    state_msgs_dict = parser.process_all_messages()

                # state_dict = checker.group_msgs_by_topic(state_msgs_dict)

                if scheduler.campaign == Campaign.RND_REPEATED:
                    # for repeated campaign
                    state_dict_list.append(state_msgs_dict)

                # state_monitor.retrieve_states(exec_cnt)
                # while True:
                #     # loop until ros2 bag play terminates
                #     if state_monitor.rosbag_proc is None:
                #         state_monitor.play_rosbag(exec_cnt)
                #     elif state_monitor.rosbag_proc.poll() is not None:
                #         print("[state monitor] rosbag play done")
                #         break

                #     if len(state_monitor.msg_queue) == 0:
                #         # prevent timing out before receiving the first msg
                #         # TODO: test with very small rosbags as subscription
                #         #       often runs pretty slow
                #         rclpy.spin_once(state_monitor)
                #     else:
                #         rclpy.spin_once(state_monitor, timeout_sec=5)

                # # state_msgs = checker.retrieve_states(exec_cnt)
                # state_dict = checker.group_msgs_by_topic(
                #     state_monitor.msg_queue
                # )
                # for topic in state_dict:
                #     print(topic, len(state_dict[topic]))
                # state_monitor.msg_queue = list()
                # state_monitor.rosbag_proc = None

                # print("run checks")
                errs = checker.run_checks(fuzzer.config, msg_list,
                        state_msgs_dict, fbk_list)
                errs = list(set(errs))
                # TODO: bring error logging and is_interesting here

                if fuzzer.config.px4_sitl:
                    if collision_checker.found_collision():
                        col_topics = list(
                            collision_checker.collision_events.keys()
                        )
                        print(f"{c.RED}COLLISION: {col_topics}{c.END}")
                        errs.append(col_topics)

                if fuzzer.config.test_rcl:
                    # API cross-check here
                    api_checker = checker.APIChecker(
                        fuzzer.config.test_rcl_feature,
                        fuzzer.config.test_rcl_targets,
                        fuzzer.config.test_rcl_job,
                    )

                    err = api_checker.check_deviant()
                    if err:
                        print(
                            f"{c.RED}{err}: {fuzzer.config.test_rcl_feature}{c.END}"
                        )
                        errs.append(err)
                        for i in range(len(fuzzer.config.test_rcl_targets)):
                            shutil.copyfile(
                                f"out-{i}",
                                os.path.join(
                                    fuzzer.config.error_dir,
                                    f"error-{frame}-out-{i}",
                                ),
                            )
                            shutil.copyfile(
                                f"trace-{i}",
                                os.path.join(
                                    fuzzer.config.error_dir,
                                    f"error-{frame}-trace-{i}",
                                ),
                            )

            if errs:
                err_file = os.path.join(
                    fuzzer.config.error_dir, f"error-{frame}"
                )

                with open(err_file, "a") as fp:
                    fp.write(str(errs))

                for exec_cnt in range(repeat):
                    # copy rosbags to {log_dir}/rosbags/{frame}/
                    bag_dir = f"states-{exec_cnt}.bag"
                    os.makedirs(os.path.join(fuzzer.config.rosbag_dir, frame))
                    shutil.copytree(
                        bag_dir,
                        os.path.join(fuzzer.config.rosbag_dir, frame, bag_dir)
                    )

                for fbk in fbk_list:
                    fbk.reset()

            else:
                print("[+] no error found")

            if scheduler.campaign == Campaign.RND_REPEATED:
                rpt_errs = checker.run_rpt_checks(
                    fuzzer.config, state_dict_list
                )

                if rpt_errs:
                    err_file = os.path.join(
                        fuzzer.config.error_dir, f"error-{frame}"
                    )

                    with open(err_file, "a") as fp:
                        fp.write(str(set(rpt_errs)))

            is_interesting = False
            for fbk in fbk_list:
                # check if any feedback element is interesting
                cur_interesting = fbk.is_interesting()
                print("check is_interesting -",
                    fbk.name,
                    cur_interesting,
                    "({})".format(fbk.value)
                )
                is_interesting = is_interesting or cur_interesting

            if is_interesting:
                print("[feedback] INTERESTING!")
                msg_to_queue = msg_list[0]

                if scheduler.campaign == Campaign.RND_SEQUENCE:
                    # takes the entire list
                    msg_to_queue = msg_list

                fuzzer.queue.append(deepcopy(msg_to_queue))

                with open(
                    os.path.join(fuzzer.config.cov_dir, frame),
                    "w",
                ) as fp:
                    for fbk in fbk_list:
                        fp.write(f"{fbk.name} {fbk.value}\n")

            if fuzzer.config.px4_sitl:

                # uncomment to collect coverage info
                # lcov_cmd = "lcov "
                # lcov_cmd += "-c "
                # lcov_cmd += "-q "
                # lcov_cmd += "-d /home/seulbae/workspace/px4-cov/build/px4_sitl_rtps "
                # lcov_cmd += "-b /home/seulbae/workspace/px4-cov/build/px4_sitl_rtps "
                # lcov_cmd += "--gcov-tool gcov "
                # lcov_cmd += "--rc lcov_branch_coverage=1 "
                # lcov_cmd += f"-o /home/seulbae/workspace/drone-sec/src/px4-coverage/lcov-{scheduler.cycle_cnt}-{scheduler.round_cnt-1}.info"

                # os.system(lcov_cmd)

                if fuzzer.config.exp_pgfuzz:
                    # cycles better be smaller
                    if scheduler.round_cnt == 8:
                        scheduler.round_cnt = 0
                        scheduler.cycle_cnt += 1
                        scheduler.is_new_cycle = True
                        print("--- cycle finished ---")
                else:
                    if scheduler.round_cnt == 5:
                        scheduler.round_cnt = 0
                        scheduler.cycle_cnt += 1
                        scheduler.is_new_cycle = True
                        print("--- cycle finished ---")

            elif fuzzer.config.test_moveit:
                if scheduler.round_cnt == 100:
                    scheduler.round_cnt = 0
                    scheduler.cycle_cnt += 1
                    scheduler.is_new_cycle = True
                    print("--- cycle finished ---")


def main(config):
    print(
        """\
                          ____
   _________  _____      / __/_  __________
  / ___/ __ \/ ___/_____/ /_/ / / /_  /_  /
 / /  / /_/ (__  )_____/ __/ /_/ / / /_/ /_
/_/   \____/____/     /_/  \__,_/ /___/___/
"""
    )

    if config.sros2:
        args = ["--ros-args", "--enclave", "/fuzzer/_fuzzer"]
    else:
        args = None

    rclpy.init(args=args)
    fuzzer = Fuzzer("_fuzzer", config)
    fuzzer.init_cov_map()
    fuzzer.init_queue()

    """
    if config.test_rosidl:
        fuzzer.init_shm_data()
    """

    # fuzzer.start_virtual_display()

    # fuzzer.init_state_monitor(config.watchlist)

    if config.px4_sitl:
        # uncomment to collect coverage info
        # lcov_cmd = "lcov "
        # lcov_cmd += "-d /home/seulbae/workspace/px4-cov/build/px4_sitl_rtps "
        # lcov_cmd += "-b /home/seulbae/workspace/px4-cov/build/px4_sitl_rtps "
        # lcov_cmd += "--zerocounters"
        # os.system(lcov_cmd)

        fuzzer.init_px4_bridge()

    if config.sros2:
        fuzzer.oh_ = OracleHandler(fuzzer, "sros2")
    # else:
    #     fuzzer.oh_ = OracleHandler(fuzzer, "turtlesim")

    # for ros graph inspection
    fuzzer.run_target(config.rospkg, config.rosnode, config.exec_cmd)

    # pp = pprint.PrettyPrinter(indent=2)

    np.random.seed(config.seed)

    if config.method == "message":
        if config.sros2:
            try:
                fuzz_targets = inspect_secure_target(fuzzer)

                if len(fuzz_targets) == 0:
                    # dealing with slow node discovery
                    mtc = ros_utils.get_msg_class_from_name(
                        "std_msgs", "String"
                    )
                    fuzz_targets.append(["/sros2_input", mtc, "/sros2_node"])

                if not config.persistent:
                    fuzzer.kill_target()

                fuzz_msg(fuzzer, fuzz_targets)

            except Exception as e:
                print("Runtime error:", str(e))
                exc_type, exc_obj, exc_tb = sys.exc_info()
                traceback.print_tb(exc_tb)

            except KeyboardInterrupt as e:
                print("Exiting on user request")

            finally:
                fuzzer.kill_target()
                fuzzer.destroy()
                rclpy.shutdown()

        else:
            try:
                fuzz_targets = inspect_target(fuzzer)

                if not config.persistent:
                    fuzzer.kill_target()

                fuzz_msg(fuzzer, fuzz_targets)

            except Exception as e:
                print("Runtime error:", str(e))
                exc_type, exc_obj, exc_tb = sys.exc_info()
                traceback.print_tb(exc_tb)

            except KeyboardInterrupt as e:
                print("Exiting on user request")

            finally:
                fuzzer.kill_target()
                fuzzer.destroy()
                rclpy.shutdown()

    elif config.method == "service":
        try:
            fuzz_srv(fuzzer)
        except Exception as e:
            print("Runtime error:", str(e))
            exc_type, exc_obj, exc_tb = sys.exc_info()
            traceback.print_tb(exc_tb)
        finally:
            fuzzer.destroy()
            rclpy.shutdown()


if __name__ == "__main__":
    argparser = argparse.ArgumentParser()

    argparser.add_argument(
        "--method",
        default="message",
        type=str,
        help="Method to test: message / service / action",
    )
    argparser.add_argument(
        "--schedule",
        default="single",
        type=str,
        help="Test scheduling: single / sequence / repeated",
    )
    argparser.add_argument(
        "--seqlen",
        default=10,
        type=int,
        help="(if schedule is sequence or repeated) number of messages in a sequence",
    )
    argparser.add_argument(
        "--repeat",
        default=1,
        type=int,
        help="Number of repetitions of a (sequence of) message",
    )
    argparser.add_argument(
        "--logdir",
        default="logs",
        type=str,
        help="dir to save messages and fuzzing metadata",
    )
    argparser.add_argument(
        "--startover",
        action="store_true",
        help="restart fuzzing from clean slate every time",
    )
    argparser.add_argument(
        "--maxloop",
        default=100,
        type=int,
        help="number of iterations, 0 for infinite",
    )
    argparser.add_argument(
        "--interval",
        default=1.0,
        type=float,
        help="interval b/w published messages in float",
    )
    argparser.add_argument(
        "--ros-pkg",
        type=str,
        help="name of ROS package to test \
                                 (e.g., turtlesim)",
    )
    argparser.add_argument(
        "--ros-node",
        type=str,
        help="name of core node to fuzz \
                                 (e.g., turtlesim_node)",
    )
    argparser.add_argument(
        "--exec-cmd",
        type=str,
        help="command to execute target system, if it is \
                                 not a ROS package",
    )
    argparser.add_argument(
        "--watchlist",
        default="watchlist/empty.json",
        type=str,
        help="path to the file containing topic watchlist",
    )
    argparser.add_argument(
        "--determ-seed",
        type=int,
        help="seed value for deterministic execution",
    )
    argparser.add_argument(
        "--fuzz-seed", type=str, help="seed file for mutation"
    )
    argparser.add_argument(
        "--px4-sitl-ros",
        action="store_true",
        help="shortcut to testing px4 drones in sitl mode (use ros)",
    )
    argparser.add_argument(
        "--px4-sitl-mav",
        action="store_true",
        help="shortcut to testing px4 drones in sitl mode (use mavlink)",
    )
    argparser.add_argument(
        "--px4-flight-mode",
        type=str,
        default="MANUAL",
        help="PX4 flight mode to test: (MANUAL/POSCTL/ALTCTL/ACRO/TAKEOFF)",
    )
    argparser.add_argument(
        "--px4-sitl-pgfuzz",
        action="store_true",
        help="shortcut to testing px4 drones with PGFUZZ style mutation",
    )
    argparser.add_argument(
        "--tb3-sitl",
        action="store_true",
        help="shortcut to testing turtlebot3 in sitl mode",
    )
    argparser.add_argument(
        "--tb3-hitl",
        action="store_true",
        help="shortcut to testing turtlebot3 in hitl mode",
    )
    argparser.add_argument(
        "--tb3-uri",
        type=str,
        default="ubuntu@192.168.0.154",
        help="(if testing hitl) ip address of TurtleBot3",
    )
    argparser.add_argument(
        "--px4-mission", default="", type=str, help="offboard mission file"
    )
    argparser.add_argument(
        "--sros2", action="store_true", help="shortcut to testing SROS2"
    )
    argparser.add_argument(
        "--no-cov",
        action="store_true",
        help="assume no coverage data (do not create shm)",
    )
    argparser.add_argument(
        "--persistent",
        action="store_true",
        help="keep target running after a round of fuzzing",
    )
    argparser.add_argument(
        "--wait",
        action="store_true",
        help="[debugging] wait after creating shm",
    )

    argparser.add_argument(
        "--test-rcl",
        action="store_true",
        help="shortcut for testing RCL API consistency",
    )
    argparser.add_argument(
        "--rcl-api",
        type=str,
        default="",
        help="(if testing rcl) specify API group to trace (e.g., publisher)",
    )
    argparser.add_argument(
        "--rcl-job",
        type=str,
        default="",
        help="(if testing rcl) specify API to execute (e.g., create_publisher)",
    )

    argparser.add_argument(
        "--test-cli",
        action="store_true",
        help="shortcut for testing RCL CLI + API consistency",
    )

    argparser.add_argument(
        "--test-rosidl",
        action="store_true",
        help="shortcut for testing ROS IDL (type system)",
    )
    argparser.add_argument(
        "--test-moveit",
        action="store_true",
        help="shortcut for testing moveit library",
    )

    args = argparser.parse_args()

    now = datetime.now()
    now_str = now.strftime("%Y%m%d-%H%M%S")
    log_dir = os.path.join(args.logdir, now_str)
    try:
        os.makedirs(log_dir)
    except FileExistsError:
        pass
    latest_link = os.path.join(args.logdir, "latest")
    if os.path.exists(latest_link):
        os.unlink(os.path.join(args.logdir, "latest"))
    os.symlink(now_str, latest_link, target_is_directory=True)

    queue_dir = os.path.join(log_dir, "queue")
    cov_dir = os.path.join(log_dir, "cov")
    error_dir = os.path.join(log_dir, "errors")
    meta_dir = os.path.join(log_dir, "metadata")
    rosbag_dir = os.path.join(log_dir, "rosbags")
    try:
        os.makedirs(queue_dir)
        os.makedirs(cov_dir)
        os.makedirs(error_dir)
        os.makedirs(meta_dir)
        os.makedirs(rosbag_dir)
    except FileExistsError:
        pass

    with open(os.path.join(log_dir, "cmd"), "w") as f:
        f.write(" ".join(sys.argv))

    with open(os.path.join(log_dir, "args"), "w") as f:
        json.dump(args.__dict__, f, indent=2)

    config = config.RuntimeConfig()
    config.method = args.method
    config.fuzz_mode = c.M_STARTOVER if args.startover else c.M_STATEFUL
    config.log_dir = log_dir
    config.queue_dir = queue_dir
    config.error_dir = error_dir
    config.cov_dir = cov_dir
    config.meta_dir = meta_dir
    config.rosbag_dir = rosbag_dir
    config.maxloop = args.maxloop
    config.interval = args.interval
    config.rospkg = args.ros_pkg
    config.rosnode = args.ros_node
    config.watchlist = args.watchlist

    if args.schedule == "single":
        config.schedule = Campaign.RND_SINGLE
        config.seqlen = 1
    elif args.schedule == "sequence":
        config.schedule = Campaign.RND_SEQUENCE
        config.seqlen = args.seqlen
    elif args.schedule == "repeated":
        config.schedule = Campaign.RND_REPEATED
        config.seqlen = args.seqlen
        assert config.seqlen > 1

    config.repeat = args.repeat

    if args.determ_seed:
        config.seed = args.determ_seed
    else:
        config.seed = int(time.time())

    if args.wait:
        config.debug_wait = True
    else:
        config.debug_wait = False

    if args.px4_sitl_ros:
        config.px4_sitl = True
        config.px4_ros = True
        config.use_mavlink = False
        config.exp_pgfuzz = False
        from px4_msgs.msg import VehicleCommand
        import px4_utils
        config.px4_mission_file = args.px4_mission
        config.flight_mode = "OFFBOARD"
    elif args.px4_sitl_mav:
        config.px4_sitl = True
        config.px4_ros = False
        config.use_mavlink = True
        config.flight_mode = args.px4_flight_mode.upper()
        config.exp_pgfuzz = False
        import px4_utils
    elif args.px4_sitl_pgfuzz:
        config.px4_sitl = True
        config.px4_ros = False
        config.use_mavlink = True
        config.exp_pgfuzz = True
        import px4_utils
        config.flight_mode = "LOITER"
    else:
        config.px4_sitl = False
        config.px4_ros = False
        config.exp_pgfuzz = False
        config.use_mavlink = False

    if args.tb3_sitl:
        config.tb3_sitl = True
    else:
        config.tb3_sitl = False

    if args.tb3_hitl:
        config.tb3_hitl = True
        config.tb3_uri = args.tb3_uri
    else:
        config.tb3_hitl = False

    if args.sros2:
        config.sros2 = True
        config.sros2_keystore = os.path.join(
            config.proj_root, "src", "fuzzing_keys"
        )
        config.sros2_enable = "true"
        config.sros2_strategy = "Enforce"
        config.sros2_enclave = "/fuzzer/sros2_node"
        config.rospkg = "sros2_node"
        config.rosnode = "sros2_node"
    else:
        config.sros2 = False

    if args.fuzz_seed:
        config.fuzz_seed = args.fuzz_seed
    else:
        config.fuzz_seed = None

    if args.no_cov:
        config.no_cov = True
    else:
        config.no_cov = False

    if args.persistent:
        config.persistent = True
    else:
        config.persistent = False

    if args.test_rcl:
        config.test_rcl = True

        valid_features = [
            "publisher",
            "subscriber",
            "service",
            "client",
            "node",
            "timer",
            "graph",
            "expand_topic_name",
            "guard_condition",
            "init",
            "time",
            "validate_topic_name",
            "wait",
        ]

        if args.rcl_api == "":
            print("--rcl-api not set. Assuming publisher")
            config.rcl_api = "publisher"
        elif args.rcl_api in valid_features:
            config.rcl_api = args.rcl_api
        else:
            print("[-] Invalid --rcl-api. Please check librcl_apis/")
            exit(1)

        valid_jobs = [
            "create_publisher",
            "create_subscriber",
            "create_node"
        ]

        # specify task (e.g., publisher, create_publisher, ...)
        # should be in sync with directory name containing target program
        if args.rcl_job == "":
            print("--rcl-job not set. Assuming create_publisher")
            config.test_rcl_job = "create_publisher"
        elif args.rcl_job in valid_jobs:
            config.test_rcl_job = args.rcl_job
        else:
            print("[-] --rcl-job is invalid or not supported.")
            exit(1)

        features_to_test = [
            config.rcl_api
        ]
        config.test_rcl_feature = features_to_test

        targets = ["py", "cpp"]# , "rs"]
        config.test_rcl_targets = targets

        # TODO:
        # - make sure rcl_lang_ws is present and compiled
        # - set path to the harness
        # - execute harness through harness.py
        #   - harness listens to data topic, executes each lang target w/ received data
        # - executor publishes msg, harness will do its job
        #   - ltrace trace dump will be generated for each lang target
        # - checker checks for inconsistency by parsing and cross-checking the trace dumps
    else:
        config.test_rcl = False

    if args.test_cli:
        config.test_cli = True
    else:
        config.test_cli = False

    if args.test_rosidl:
        # one target with multiple topics - why not
        config.test_rosidl = True

        lang = ["py", "cpp"]
        # don't need to search target
        # just make sure the harness is launched

        config.test_rosidl_lang = lang[0]
        config.schedule = Campaign.IDL_CHECK
    else:
        config.test_rosidl = False

    if args.test_moveit:
        from moveit_msgs.msg import Constraints, JointConstraint
        config.test_moveit = True
    else:
        config.test_moveit = False

    config.exec_cmd = None
    ret = config.find_package_metadata()

    if ret < 0:
        if args.exec_cmd == "px4":
            # PX4-specific
            px4_root = os.path.join(
                config.proj_root, "targets", "PX4-Autopilot"
            )
            px4_build_dir = os.path.join(px4_root, "build", "px4_sitl_rtps")
            sitl_script = os.path.join(px4_root, "Tools", "sitl_run.sh")
            sitl_opts = [
                os.path.join(px4_build_dir, "bin", "px4"),
                "none",
                "gazebo",
                "none",
                "none",
                px4_root,
                px4_build_dir,
            ]
            px4_cmd = "{} {}".format(sitl_script, " ".join(sitl_opts))
            config.exec_cmd = px4_cmd
        elif args.exec_cmd is not None:
            config.exec_cmd = args.exec_cmd
        else:
            config.exec_cmd = ""

    main(config)
