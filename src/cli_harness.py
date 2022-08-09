import os
import subprocess as sp
import signal
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CLISubscriber(Node):
    def __init__(self):
        super().__init__("cli_harness")

        # get cli command
        self.subscription = self.create_subscription(
            String, "cli_topic", self.cli_harness_callback, 10
        )

        self.targets = [
            "ros2 run examples_rclpy_minimal_publisher publisher_member_function",
            "ros2 run examples_rclcpp_minimal_publisher publisher_member_function",
        ]

    def cli_harness_callback(self, msg):
        print(f"[cli harness] Testing {msg.data}")

        # run target1, run cmd, store output1
        # run target2, run cmd, store output2
        for i, target_cmd in enumerate(self.targets):
            outfile = f"out-{i}"
            try:
                os.remove(outfile)
            except:
                pass

            pgrp = sp.Popen(
                target_cmd,
                shell=True,
                preexec_fn=os.setpgrp,
                stdout=sp.DEVNULL,
                stderr=sp.DEVNULL,
            )
            time.sleep(3)

            cmd = f"ros2 param set /minimal_publisher use_sim_time {msg.data} >{outfile} 2>&1"
            os.system(cmd)

            if pgrp is not None:
                try:
                    os.killpg(pgrp.pid, signal.SIGKILL)
                except:
                    print(f"[-] could not kill {target_cmd}")

        try:
            os.remove(".waitlock")
        except:
            pass


def main():
    rclpy.init()

    node = CLISubscriber()

    rclpy.spin_once(node, timeout_sec=20)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
