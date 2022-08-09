#!/usr/bin/python3

import time
import sys
import random

import px4_utils

import rclpy
from px4_msgs.msg import Timesync, VehicleCommand, TrajectorySetpoint, OffboardControlMode


class Node:
    def __init__(self, name):
        self.node_handle = rclpy.create_node(name)
        self.sub_timesync = self.node_handle.create_subscription(
                Timesync, "Timesync_PubSubTopic", self.timesync_callback, 10)
        self.bit = 0

    def timesync_callback(self, msg):
        self.ts = msg.timestamp
        print("ts:", self.ts)

    def init_publisher(self):
        self.pub_offboard_control_mode = self.node_handle.create_publisher(
                OffboardControlMode, "/OffboardControlMode_PubSubTopic", 10)
        self.pub_vehiclecommand = self.node_handle.create_publisher(
                VehicleCommand, "/VehicleCommand_PubSubTopic", 10)
        self.pub_trajectory = self.node_handle.create_publisher(
                TrajectorySetpoint, "/TrajectorySetpoint_PubSubTopic", 10)

    def publish_offboard_mode(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.timestamp = self.ts
        msg.param1 = 1.0
        msg.param2 = 6.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        print(msg)
        self.pub_vehiclecommand.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.ts
        msg.position = True
        msg.velocity = True
        msg.acceleration = True
        msg.attitude = True
        msg.body_rate = True
        print(msg)
        self.pub_offboard_control_mode.publish(msg)


    def publish_arm_command(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.timestamp = self.ts
        msg.param1 = 1.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        print(msg)
        self.pub_vehiclecommand.publish(msg)

    def publish_trajectory(self, i, home=False, msg=None):

        if msg is None:
            # manual testing
            msg = TrajectorySetpoint()
            msg.timestamp = self.ts

            # msg.x = float(random.randint(-1000, 1000)) / random.randint(1, 5)
            # msg.y = float(random.randint(-1000, 1000)) / random.randint(1, 5)
            # msg.z = float(random.randint(-100, -1)) / random.randint(1, 5)
            if home:
                msg.x = 0.0
                msg.y = 0.0
                msg.z = -5.0
                msg.yaw = 0.0

            else:
                msg.x = 10.0
                msg.y = 10.0
                msg.z = -5.0
                if i == 50:
                    print("**************************************************************************")
                    print("**************************************************************************")
                    print("**************************************************************************")
                    print("**************************************************************************")
                    print("**************************************************************************")
                    print("**************************************************************************")
                    msg.vx = 0.0
                    msg.acceleration[0] = 0.0
                    # msg.vx = 0.0
                    # msg.acceleration[0] = 90000000.0
                else:
                    msg.vx = 0.0
                msg.vy = 0.0
                # msg.acceleration[0] = 3000.0

                # if self.bit:
                    # msg.yaw = -3.14
                # else:
                    # msg.yaw = 0.0
                # self.bit ^= 1
                msg.yaw = 3.14
                msg.yawspeed = 0.0

        print(msg)
        self.pub_trajectory.publish(msg)


def main(msg_list):

    rclpy.init()
    node = Node("sender")
    node.init_publisher()

    # https://docs.px4.io/master/en/flight_modes/offboard.html#offboard-mode
    # The vehicle must be already be receiving a stream of target setpoints
    # (>2Hz) before this mode can be engaged.
    for i in range(10):
        rclpy.spin_once(node.node_handle)
        node.publish_offboard_control_mode()
        node.publish_trajectory(i)
        time.sleep(0.1)

    rclpy.spin_once(node.node_handle)
    node.publish_offboard_mode()
    time.sleep(0.1)

    # The vehicle must be armed before this mode can be engaged.
    rclpy.spin_once(node.node_handle)
    node.publish_arm_command()
    time.sleep(0.1)

    # The vehicle will exit the mode if target setpoints are not received at a
    # rate of > 2Hz.

    if msg_list:
        # given a mission
        for msg in msg_list:
            rclpy.spin_once(node.node_handle)
            node.publish_offboard_control_mode()
            node.publish_trajectory(i=0, home=False, msg=msg)
            time.sleep(0.1)

    else:
        # manual testing
        for i in range(100):
            rclpy.spin_once(node.node_handle)
            node.publish_offboard_control_mode()
            node.publish_trajectory(i)
            time.sleep(0.1)

        for i in range(100):
            rclpy.spin_once(node.node_handle)
            node.publish_offboard_control_mode()
            node.publish_trajectory(i, home=True)
            time.sleep(0.1)

    node.node_handle.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    trajectory_msg_list = None

    if len(sys.argv) > 1:
        trajectory_msg_list = px4_utils.read_trajectory_seed(sys.argv[1])
        if trajectory_msg_list is None:
            print("[-] wrong file")
            sys.exit(1)

    main(trajectory_msg_list)
