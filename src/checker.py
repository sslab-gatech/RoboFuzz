import subprocess as sp
import threading
import os
import pickle
import time
import math
import json
import statistics

import numpy as np

import constants as c

import rclpy
from rclpy.node import Node

import ros_utils

# import checkers
import oracles.turtlesim
import oracles.turtlebot
import oracles.px4
import oracles.moveit
import oracles.rosidl


class StateMonitorNode(Node):
    subs = dict()
    msg_queue = list()
    rosbag_proc = None

    def __init__(self, fuzzer):
        super().__init__("_checker")

        self.fuzzer = fuzzer

    def retrieve_states(self, exec_cnt):
        """
        Retrieve recorded states by re-playing a rosbag file
        """
        print("[checker] retrieving states")

        watchlist_file = self.fuzzer.config.watchlist

        with open(watchlist_file, "r") as fp:
            data = fp.read()

        self.watchlist = json.loads(data)

        for target_topic in self.watchlist:
            topic_msg_type = self.watchlist[target_topic]
            pkg_name = ".".join(topic_msg_type.split("/")[:-1])
            module_name = topic_msg_type.split("/")[-1]

            msg_type_class = ros_utils.get_msg_class_from_name(
                pkg_name.split(".")[0], module_name
            )

            self.subs[target_topic] = self.create_subscription(
                msg_type_class, target_topic, self.msg_callback, 500
            )

            print(f"[state monitor] subscribed to {target_topic}")
            time.sleep(1)

        # self.play_rosbag(exec_cnt)

    def msg_callback(self, msg):
        """
        Generic callback for storing re-played messages of any topic
        """

        self.msg_queue.append(msg)

    def play_rosbag(self, exec_cnt):
        rosbag_cmd = f"ros2 bag play states-{exec_cnt}.bag"

        self.rosbag_proc = sp.Popen(
            rosbag_cmd, shell=True, stdout=sp.PIPE, stderr=sp.PIPE
        )
        print("[checker] started ros2 bag play:", rosbag_cmd)

    def check_rosbag(self):
        # check if ros2 bag play is still running
        pass


def group_msgs_by_topic(states):
    """
    input: queue of messages of various topics
    output: dictionary mapping each topic to corresponding messages
    """

    topic_to_msg_dict = {}
    for msg in states:
        msg_type_name = f"{msg.__class__.__module__}.{msg.__class__.__name__}"
        try:
            topic_to_msg_dict[msg_type_name].append(msg)
        except KeyError:
            topic_to_msg_dict[msg_type_name] = [msg]

    return topic_to_msg_dict


def retrieve_states():
    state_msgs = pickle.load(open("states.pkl", "rb"))
    print("recorded:", len(state_msgs))

    try:
        os.remove("states.pkl")
    except:
        print("[-] cannot remove states file")
    # print("[monitor] retrieved states")

    return state_msgs


def run_checks(config, msg_list, state_dict, feedback_list=[]):
    """invoke checks of the oracles defined for each target package

    Parameters
    ----------
    config : RuntimeConfig object
    msg_list : list of messages published
    state_dict : dict of messages captured by pre-defined state topics
    feedback_list : list of Feedback objects
    """

    errs = []
    if config.rospkg == "turtlesim":
        try:
            pose_list = state_dict["/turtle1/pose"]
        except KeyError:
            print("[checker] no Pose available")
            return errs

        errs = oracles.turtlesim.check(config, msg_list, pose_list, feedback_list)

    elif config.tb3_sitl or config.tb3_hitl:
        if (
            "/imu" not in state_dict
            and "/odom" not in state_dict
            and "/scan_list" not in state_dict
        ):
            print("[checker] no state available")
            return errs

        errs = oracles.turtlebot.check(config, msg_list, state_dict, feedback_list)

    elif config.px4_sitl:
        if (
            "/ActuatorArmed_PubSubTopic" not in state_dict
            and "/ActuatorOutputs_PubSubTopic" not in state_dict
            and "/BatteryStatus_PubSubTopic" not in state_dict
            and "/SensorAccel_PubSubTopic" not in state_dict
            and "/SensorBaro_PubSubTopic" not in state_dict
            and "/SensorGps_PubSubTopic" not in state_dict
            and "/SensorGyro_PubSubTopic" not in state_dict
            and "/SensorsStatusImu_PubSubTopic" not in state_dict
            and "/SensorCombined_PubSubTopic" not in state_dict
            and "/VehicleOdometry_PubSubTopic" not in state_dict
            and "/VehicleImu_PubSubTopic" not in state_dict
            and "/VehicleGpsPosition_PubSubTopic" not in state_dict
            and "/VehicleLocalPosition_PubSubTopic" not in state_dict
            and "/VehicleGlobalPosition_PubSubTopic" not in state_dict
            and "/VehicleAttitude_PubSubTopic" not in state_dict
            and "/VehicleAcceleration_PubSubTopic" not in state_dict
            and "/VehicleAngularAcceleration_PubSubTopic" not in state_dict
            and "/VehicleAngularVelocity_PubSubTopic" not in state_dict
        ):
            print("[checker] no state available")
            return errs

        errs = oracles.px4.check(config, msg_list, state_dict, feedback_list)

    elif config.test_moveit:
        if (
            "/joint_states" not in state_dict
            and "/panda_arm_controller/state" not in state_dict
            and "/move_action/_action/status" not in state_dict
            and "/motion_plan_request" not in state_dict
        ):
            print("[checker] no state available")
            return errs

        errs = oracles.moveit.check(config, msg_list, state_dict, feedback_list)

    elif config.test_rosidl:
        if len(msg_list) != 1:
            print("[-] msg_list contains more than one message")
            exit(1)

        errs = oracles.rosidl.check(config, msg_list, state_dict, feedback_list)

    elif config.test_cli:
        try:
            with open("out-0", "r") as f:
                out0 = "\n".join(f.readlines())
        except:
            print("[checker] out-0 does not exist")
            exit(1)

        try:
            with open("out-1", "r") as f:
                out1 = "\n".join(f.readlines())
        except:
            print("[checker] out-1 does not exist")
            exit(1)

        if out0 != out1:
            errs = [f"CLI outputs do not match: {out0} vs {out1}"]

    # Print result
    if len(errs) > 0:
        if len(errs) > 10:
            print(f"{c.RED}ERROR DETECTED: {len(errs)}")
            for err in list(set(errs))[:10]:
                print("-", err)
            print("- ...")
            print(c.END)
        else:
            print(f"{c.RED}ERROR DETECTED: {len(errs)}")
            for err in list(set(errs)):
                print("-", err)
            print(c.END)

    return errs


def run_rpt_checks(config, state_dict_list):
    """check if repeated executions over the same messages result in the identical states"""

    print(
        f"[checker] cross-checking states from {len(state_dict_list)} executions"
    )

    last_states = dict()

    for state_dict in state_dict_list:
        for topic in state_dict:
            last_state = state_dict[topic][-1]
            if topic not in last_states:
                last_states[topic] = [last_state]
            else:
                last_states[topic].append(last_state)

    errs = []

    for topic in last_states:
        last_msgs = last_states[topic]
        num_last_states = len(last_msgs)
        if num_last_states != len(state_dict_list):
            errs.append(f"number of topics do not match: {topic}")

        # TODO: cross-check messages in last_msgs

    return errs


class APIChecker:
    def __init__(self, feature, targets, job):
        self.tracer = "ltrace"
        self.feature = feature
        self.targets = targets
        self.job = job

    def config_trace(self):
        pass

    def get_timestamp(self, filename="out"):
        if not os.path.isfile(filename):
            print(f"[tracer] Cannot find trace file {filename}")
            return -1

        with open(filename, "r") as fp:
            out_raw = fp.readlines()

        t0 = -1
        t1 = 2147483647.0
        for line in out_raw:
            if "time1" in line:
                t0 = line.strip().split(" ")[-1]
                if "." not in t0:
                    t0 = (
                        int(t0) / 1000000.0
                    )  # c++/rust prints us w/o decimal point
                else:
                    t0 = float(t0)
            elif "time2" in line:
                t1 = line.strip().split(" ")[-1]
                if "." not in t1:
                    t1 = (
                        int(t1) / 1000000.0
                    )  # c++/rust prints us w/o decimal point
                else:
                    t1 = float(t1)
                break

        self.t0 = t0
        self.t1 = t1

    def parse_trace(self, filename="trace"):
        if not os.path.isfile(filename):
            print(f"[tracer] Cannot find trace file {filename}")
            return -1

        func_dict = dict()

        with open(filename, "r") as fp:
            trace_raw = fp.readlines()

        for line in trace_raw:
            line = line.strip()
            timestamp = float(line.split(" ")[0])

            if timestamp < self.t0 or timestamp > self.t1:
                continue

            if "<unfinished ...>" in line:
                func_name = line.split(" ")[1].split("@")[0]
                retval = None
                if func_name in func_dict:
                    func_dict[func_name].append([timestamp, retval])
                else:
                    func_dict[func_name] = [[timestamp, retval]]

            elif "<..." in line and "resumed>" in line:
                func_name = line.split(" ")[2]
                retval = line.split("= ")[-1]

                if func_name not in func_dict:
                    print(f"[tracer] fatal error: {func_name} not seen yet")
                    return -1

                # update retval
                func_dict[func_name][-1][1] = retval

            elif ".so" in line:
                func_name = line.split(" ")[1].split("@")[0]
                retval = line.split("= ")[-1]

                if func_name in func_dict:
                    func_dict[func_name].append([timestamp, retval])
                else:
                    func_dict[func_name] = [[timestamp, retval]]

        # print(func_dict)
        return func_dict

    def check_deviant(self):
        print("[API checker] Checking deviant API behavior")
        target_trace_map = dict()

        for i, target in enumerate(self.targets):
            outfile = f"out-{i}"
            tracefile = f"trace-{i}"
            ret = self.get_timestamp(outfile)
            func_dict = self.parse_trace(tracefile)
            target_trace_map[target] = func_dict

        # 1. check if each lang calls the same API
        functions_list = list()
        target_list = target_trace_map.keys()
        for target in target_list:
            func_dict = target_trace_map[target]
            functions = list(func_dict.keys())
            functions.sort()
            functions_list.append(functions)

        if not functions_list.count(functions_list[0]) == len(functions_list):
            err = "RCL API discrepancy - called APIs do not match"
            for f in functions_list:
                print(f)
            return err

        # 2. check retval
        function_retvals_map = dict()
        for target in target_list:
            func_dict = target_trace_map[target]
            functions = list(func_dict.keys())

            for function in functions:
                ret_list = [x[1] for x in func_dict[function]]
                if function in function_retvals_map:
                    function_retvals_map[function].append(ret_list)
                else:
                    function_retvals_map[function] = [ret_list]

        for function in function_retvals_map:
            retvals_list = function_retvals_map[function]
            if not retvals_list.count(retvals_list[0]) == len(retvals_list):
                err = "RCL API discrepancy - return values do not match"
                print(function_retvals_map)
                return err


class CollisionChecker:
    topic_proc_map = dict()
    # topic_stdout_map = dict()
    collision_events = dict()
    topic_monitor_map = dict()

    def __init__(self):
        pass

    def listen(self, topics):
        print("[collision checker] start listening")
        self.collision_events = dict()
        for topic in topics:
            # print("[collision checker] start listening to ", topic)

            cmd = ["gz", "topic", "-u", "-e", topic]
            proc = sp.Popen(cmd, shell=False, stdout=sp.PIPE, stderr=sp.PIPE)

            t = threading.Thread(
                target=self.monitor,
                args=(
                    proc,
                    topic,
                ),
            )
            t.start()

            self.topic_proc_map[topic] = proc
            self.topic_monitor_map[topic] = t

    def stop(self):
        print("[collision checker] stop listening")
        for topic in self.topic_proc_map:
            # print("[collision checker] stop listening to", topic)

            proc = self.topic_proc_map[topic]
            proc.kill()

            thread = self.topic_monitor_map[topic]
            thread.join()
            assert thread.is_alive() is False

            # self.topic_stdout_map[topic] = outs

    def monitor(self, proc, topic):
        for line in iter(proc.stdout.readline, b""):
            if b"contact" in line:
                line_str = line.decode("utf-8")
                # print("collision: {0}".format(line_str), end="")

                try:
                    self.collision_events[topic].append(line_str)
                except KeyError:
                    self.collision_events[topic] = [line_str]

    def found_collision(self):

        if len(self.collision_events) > 0:
            return True

        # for topic in self.topic_stdout_map:
        # outs = self.topic_stdout_map[topic]
        # if b'contact' in outs:
        # self.collision_topic.append(topic)
        # collision = True

        # return collision

    def print_collision_topics(self) -> None:

        print(list(self.collision_events.keys()))
