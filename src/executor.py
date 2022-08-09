import os

from threading import Thread
import time
import pickle
from enum import Enum, auto
import subprocess as sp
import json
import signal
import shutil

import constants as c
import ros_utils

import rclpy
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy
from rosidl_runtime_py import message_to_ordereddict, message_to_yaml

from std_msgs.msg import UInt64
from std_srvs.srv import Empty


class ExecMode(Enum):
    SINGLE = auto()
    SEQUENCE = auto()


class Executor:
    def __init__(self, fuzzer):
        self.fuzzer = fuzzer

    def prep_execution(self, msg_type_class, topic_name):
        # prepare execution
        # create publisher

        self.topic_name = topic_name
        self.msg_type_class = msg_type_class

        self.msg_typestr = ros_utils.get_msg_typename_from_class(
            self.msg_type_class
        )

        # self.fuzzer.pub = self.fuzzer.node_ptr.create_publisher(
        #     msg_type_class, topic_name, 100
        # )

        # qos = QoSProfile(
        #     history=HistoryPolicy.KEEP_ALL,
        #     durability=DurabilityPolicy.TRANSIENT_LOCAL,
        # )
        # self.fuzzer.state_pub = self.fuzzer.node_ptr.create_publisher(
        #     Bool, "listen_flag", qos_profile=qos
        # )

        # self.fuzzer.cli_start_monitor = self.fuzzer.node_ptr.create_client(
        #     Empty,
        #     "start_monitor"
        # )
        # while not self.fuzzer.cli_start_monitor.wait_for_service(timeout_sec=1.0):
        #     print("[executor] waiting for start_monitor service")

        # # self.fuzzer.req_start_monitor = Empty.Request()

        # self.fuzzer.cli_stop_monitor = self.fuzzer.node_ptr.create_client(
        #     Empty,
        #     "stop_monitor"
        # )
        # while not self.fuzzer.cli_stop_monitor.wait_for_service(timeout_sec=1.0):
        #     print("[executor] waiting for stop_monitor service")

        # # self.fuzzer.req_stop_monitor = Empty.Request()

    def clear_execution(self):
        return
        self.fuzzer.pub.destroy()
        self.fuzzer.state_pub.destroy()

    def request_start_monitor(self):
        future = self.fuzzer.cli_start_monitor.call_async(Empty.Request())

        # this shouldn't timeout, but it sometime does.
        rclpy.spin_until_future_complete(
            self.fuzzer.node_ptr, future, timeout_sec=None
        )

        print("[executor] monitor started")
        # res = future.result() # don't have to check

    def request_stop_monitor(self):
        print("[executor] request stop monitor")
        future = self.fuzzer.cli_stop_monitor.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self.fuzzer.node_ptr, future)
        # spin_thread = Thread(target=rclpy.spin, args=(self.fuzzer.node_ptr,))
        # spin_thread.start()
        # res = self.fuzzer.cli_stop_monitor.call(Empty.Request())
        print("[executor] monitor stopped")
        # res = future.result() # don't have to check

    def start_watching(self):
        print("[executor] start watching")

        with open("/tmp/start_ts", "w") as f:
            f.write(str(int(str(time.time()).replace(".", ""))))

        # sp.call(
            # [
                # "ros2",
                # "topic",
                # "pub",
                # "--once",
                # "/listen_flag",
                # "std_msgs/UInt64",
                # "{data: " + str(int(str(time.time()).replace(".", ""))) + "}",
            # ],
            # stdout=sp.DEVNULL,
            # stderr=sp.DEVNULL,
        # )

        return

        state_msg = Bool()
        state_msg.data = True

        # deal with lossy environments by repeating msgs
        for i in range(5):
            self.fuzzer.state_pub.publish(state_msg)
            time.sleep(0.5)

    def stop_watching(self):
        print("[executor] stop watching")

        with open("/tmp/end_ts", "w") as f:
            f.write(str(int(str(time.time()).replace(".", ""))))

        # sp.call(
            # [
                # "ros2",
                # "topic",
                # "pub",
                # "--once",
                # "/listen_flag",
                # "std_msgs/UInt64",
                # "{data: " + str(int(str(time.time()).replace(".", ""))) + "}",
            # ],
            # stdout=sp.DEVNULL,
            # stderr=sp.DEVNULL,
        # )

        return

        state_msg = Bool()
        state_msg.data = False

        # deal with lossy environments by repeating msgs
        for i in range(5):
            self.fuzzer.state_pub.publish(state_msg)
            time.sleep(0.5)

    def start_rosbag(self, exec_cnt):
        watchlist_file = self.fuzzer.config.watchlist

        bag_dir = f"states-{exec_cnt}.bag"
        with open(watchlist_file, "r") as fp:
            data = fp.read()

        try:
            shutil.rmtree(bag_dir)
        except:
            print("[executor] could not remove previous bag dir")

        rosbag_cmd = f"ros2 bag record -o {bag_dir} --include-hidden-topics --qos-profile-overrides-path qos_override.yaml "
        watchlist = json.loads(data)
        for target in watchlist:
            rosbag_cmd += target + " "
        rosbag_cmd += "listen_flag"

        self.rosbag_pgrp = sp.Popen(
            rosbag_cmd,
            shell=True,
            preexec_fn=os.setpgrp,
            stdout=sp.PIPE,
            stderr=sp.PIPE,
        )
        print("[executor] started ros2 bag recording")

        # need time for topics to be discovered
        time.sleep(1)

    def kill_rosbag(self):
        try:
            print("[executor] killing ros2 bag")
            os.killpg(self.rosbag_pgrp.pid, signal.SIGINT)
        except ProcessLookupError as e:
            print("[-] ros2 bag killpg error:", e)
        except AttributeError as e:
            print("[-] ros2 bag not initialized:", e)

    def save_msg_to_queue(self, msg, frame, subframe):
        if self.fuzzer.config.replay:
            return

        md = message_to_ordereddict(msg)
        queue_file = os.path.join(
            self.fuzzer.config.queue_dir, "msg-{}-{}".format(frame, subframe)
        )

        pickle.dump(md, open(queue_file, "wb"))

    def do_pre_execution(self, pre_exec_list):
        """
        Execute tasks (if any) that need to precede the main execution
        (e.g., arming)

        Parameters
        ----------
        pre_exec_list : list of tuples (function, (args))
        """

        if pre_exec_list is not None:
            for (function, args) in pre_exec_list:
                function(*args)

    def do_post_execution(self, post_exec_list):
        """
        Execute tasks (if any) that need to precede the main execution
        (e.g., arming)

        Parameters
        ----------
        post_exec_list : list of tuples (function, (args))
        """

        if post_exec_list is not None:
            for (function, args) in post_exec_list:
                function(*args)

    def do_ros2_pub(self, msg):
        try:
            os.remove("out")
        except:
            pass

        try:
            os.remove("err")
        except:
            pass

        out = open("out", "w")
        err = open("err", "w")
        sp.call(
            [
                "ros2",
                "topic",
                "pub",
                "--once",
                self.topic_name,
                self.msg_typestr,
                message_to_yaml(msg),
            ],
            stdout=out,
            stderr=err,
        )
        out.close()
        err.close()

        if os.path.getsize("err") > 0:
            with open("err", "r") as f:
                e = "".join(f.readlines())
            return e

    def execute(
        self,
        mode,
        msg_list,
        frame,
        frequency,
        repeat=1,
        pre_exec_list=None,
        post_exec_list=None,
        pub_function=None,
        wait_lock=None,
    ):
        """
        Run target,
        publish a message (or a sequence of messages) to a topic,
        and then kill target.

        Parameters
        ----------
        mode : ExecMode
        msg_list : [message_type]
        frame : datetime
            Timeframe the message is mutated
        frequency : float
            Rate (Hz) at which the given messages are published
        repeat : int
            Number of desired repetitions of the entire message (default: 1)
            Can be used for measuring repeatability by setting repeat > 1
        pre_exec_list : [function]
            List of tasks that need to precede the main execution
            (e.g., px4 arming)
        pub_function : function
            Custom method to publish messages. If None specified, use roscli.
        """

        exec_cnt = 0
        period = 1 / frequency

        print("[executor] len(msg_list):", len(msg_list))

        if mode == ExecMode.SINGLE:
            assert len(msg_list) == 1

        elif mode == ExecMode.SEQUENCE:
            assert len(msg_list) > 1

        while True:  # outer loop: repetition of entire messages
            if exec_cnt == repeat:
                print(f"finished repeating {repeat} times")
                break

            print(f"repeating {exec_cnt}-th sequence")

            if not self.fuzzer.config.persistent:
                # target should already be running in persistent mode
                self.fuzzer.run_target(
                    self.fuzzer.config.rospkg,
                    self.fuzzer.config.rosnode,
                    self.fuzzer.config.exec_cmd,
                )

            self.do_pre_execution(pre_exec_list)

            # fp = open(c.WATCHFLAG, "w")
            # fp.close()
            self.start_rosbag(exec_cnt)
            self.start_watching()

            # self.request_start_monitor()
            # print("[executor] waiting for state monitor", end="")
            # while not os.path.isfile(c.PUBLOCK):
            #     print(".", end="", flush=True)
            #     time.sleep(0.1)
            # print()

            pub_failure = False
            if mode == ExecMode.SINGLE:
                subframe = time.time()
                msg = msg_list[0]

                self.save_msg_to_queue(msg, frame, subframe)

                # TODO: publishing through API is terribly unreliable.
                # Switch to externally calling "ros2 topic pub" as it
                # automatically creates a new hidden publisher.

                # byte doesn't work well with message_to_yaml()
                # reported the issue here:
                # https://github.com/ros2/rosidl_runtime_py/issues/14
                if pub_function:
                    pub_function(msg)

                else:
                    ret = self.do_ros2_pub(msg)

                    if ret:
                        pub_failure = True
                        pub_failure_msg = ret
                        print(f"[executor] failed to publish: {ret}")


                # self.fuzzer.pub.publish(msg)
                time.sleep(period)

            elif mode == ExecMode.SEQUENCE:
                for i, msg in enumerate(msg_list):
                    # print("{}-th msg in sequence".format(i))
                    subframe = time.time()

                    self.save_msg_to_queue(msg, frame, subframe)

                    if pub_function:
                        pub_function(msg)

                    else:
                        ret = self.do_ros2_pub(msg)

                        if ret:
                            pub_failure = True
                            pub_failure_msg = ret
                            print(f"[executor] failed to publish: {ret}")

                    # self.fuzzer.pub.publish(msg)
                    time.sleep(period)

            if wait_lock:
                f = open(wait_lock, "w")
                f.close()

                while True:
                    time.sleep(0.2)
                    if not os.path.isfile(wait_lock):
                        break

            exec_cnt += 1

            # self.request_stop_monitor()
            # try:
            #     print("removing watchflag")
            #     os.remove(c.WATCHFLAG)
            # except:
            #     print("cannot remove watchflag file")
            #     pass

            # # wait for the monitor to dump file
            # while not os.path.isfile("states.pkl"):
            #     time.sleep(0.1)
            self.do_post_execution(post_exec_list)

            self.stop_watching()
            self.kill_rosbag()

            if not self.fuzzer.config.persistent:
                # should not kill target in persistent mode
                self.fuzzer.kill_target()

            if pub_failure:
                return (1, pub_failure_msg)
            else:
                return (0, "")
