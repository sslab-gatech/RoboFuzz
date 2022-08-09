"""
ROS Fuzzer example test cases module.

:authors: Alias Robotics S.L. Borja Erice, Odei Olalde, Xabi Perez, Gorka Olalde
"""
import unittest

import rclpy
from hypothesis import given
from rcl_interfaces.msg import ParameterEvent
from rclpy.node import Node
from std_msgs.msg import String

from ros2_fuzzer.process_handling import FuzzedNodeHandler
from ros2_fuzzer.ros_commons import map_ros_types


class TestRosListener(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = Node('fuzzer_node')
        self.pub = self.node.create_publisher(ParameterEvent, '/chatter')
        self.health_checker = FuzzedNodeHandler('/listener')

    def tearDown(self):
        rclpy.shutdown('Shutting down fuzzer node')

    @given(message=map_ros_types(String))
    def test_message(self, message):
        self.pub.publish(message)
        assert self.health_checker.check_if_alive()


