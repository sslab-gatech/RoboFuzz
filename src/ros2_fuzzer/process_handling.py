"""
ROS Fuzzer process health handling module.

:authors: Alias Robotics S.L. Borja Erice, Odei Olalde, Xabi Perez, Gorka Olalde
"""
import rclpy


class FuzzedNodeHandler:

    def __init__(self, node_name):
        self.node_name = node_name
        try:
            rclpy.init()
        except:
            pass
        self.node = rclpy.create_node('health_checker')

    def check_if_alive(self):
        nodes, namespaces = self.node.get_node_names()
        return self.node_name in nodes

