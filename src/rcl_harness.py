import os
import sys
import time
import subprocess as sp
import signal
import argparse

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from std_msgs.msg import String
from tracer import APITracer


def parse_cov(covfile):
    with open(covfile, "r") as f:
        line = f.read()

    branches = set(line.split(" "))

    return branches


def test_cov_diff():
    cwd = os.getcwd()
    rcl_target_dir = os.path.join(cwd, "../targets/rcl_lang_ws/build")

    targets = [
        "python3 "
        + os.path.join(
            rcl_target_dir,
            "rclpy_publisher/rclpy_publisher/rclpy_publisher.py",
        ),
        os.path.join(rcl_target_dir, "rclcpp_publisher/rclcpp_publisher"),
        os.path.join(
            rcl_target_dir,
            "rclrs_publisher/ament_cargo/rclrs_publisher/target/release",
        ),
    ]

    for i, target in enumerate(targets):
        cmd = "COVFILE_PATH=./cov-{} {}".format(i, target)
        print(target)
        os.system(cmd)

    branches = []
    for i in range(len(targets)):
        branches.append(parse_cov(f"cov-{i}"))

    a = branches[0]
    b = branches[1]

    c = a.difference(b)
    d = b.difference(a)
    u = a.union(b)
    i = a.intersection(b)

    print("union", len(u))
    print("intersection", len(i))
    print("c", len(c))
    print("d", len(d))


class RCLSubscriber(Node):
    def __init__(self, features, targets, job):
        super().__init__("rcl_harness")
        self.subscription = self.create_subscription(
            String, "rcl_topic", self.rcl_harness_callback, 10
        )
        self.features = features
        self.job = job

        cwd = os.getcwd()
        rcl_target_dir = os.path.join(cwd, "../targets/rcl_lang_ws/build")

        self.api_list = []
        for feature in features:
            with open(f"librcl_apis/{feature}.txt", "r") as f:
                self.api_list.extend(f.readlines())

        for i in range(len(self.api_list)):
            self.api_list[i] = self.api_list[i].strip() + "@librcl.so"

        # set blacklist, whitelist

        self.targets = []
        if "py" in targets:
            self.targets.append(
                "python3 "
                + os.path.join(
                    rcl_target_dir,
                    f"rclpy_{job}/rclpy_{job}/rclpy_{job}.py",
                )
            )
        if "cpp" in targets:
            self.targets.append(
                os.path.join(rcl_target_dir, f"rclcpp_{job}/rclcpp_{job}")
            )
        if "rs" in targets:
            self.targets.append(
                os.path.join(
                    rcl_target_dir,
                    f"rclrs_{job}/ament_cargo/rclrs_{job}/target/release/rclrs_{job}",
                )
            )

    def rcl_harness_callback(self, msg):
        print(f"[rcl harness] Testing {self.features}")

        self.api_list.insert(0, "")
        lib_filter = " -x ".join(self.api_list)
        self.api_list.remove("")

        pgrp = None
        if "subscriber" in self.features:
            # need to re-echo data to topic as subscribers are waiting
            cmd = "ros2 topic pub /topic std_msgs/String '{data: %s}'" % (
                msg.data
            )
            pgrp = sp.Popen(
                cmd,
                shell=True,
                preexec_fn=os.setpgrp,
                stdout=sp.DEVNULL,
                stderr=sp.DEVNULL,
            )

        # exec each rcl lang binaries
        for i, target in enumerate(self.targets):
            print(f"[rcl harness] Executing {target}")

            outfile = f"out-{i}"
            tracefile = f"trace-{i}"
            try:
                os.remove(outfile)
            except:
                pass
            try:
                os.remove(tracefile)
            except:
                pass

            cmd = f"ltrace -o {tracefile} -ttt {lib_filter} {target} {msg.data} > {outfile}"
            os.system(cmd)

        if pgrp is not None:
            try:
                os.killpg(pgrp.pid, signal.SIGKILL)
            except:
                print("[-] could not kill ros2 topic pub")

        try:
            os.remove(".waitlock")
        except:
            pass


def main(features, targets, job):
    rclpy.init()

    node = RCLSubscriber(features, targets, job)

    rclpy.spin_once(node, timeout_sec=20)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    argparser = argparse.ArgumentParser()

    argparser.add_argument(
        "-f",
        type=str,
        nargs="+",
    )
    argparser.add_argument(
        "-t",
        type=str,
        nargs="+",
    )
    argparser.add_argument(
        "-j",
        type=str,
    )

    args = argparser.parse_args()

    main(args.f, args.t, args.j)
