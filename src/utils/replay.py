#!/usr/bin/python3

import os
import sys
import pickle
import time
from functools import reduce
import importlib

from ros_utils import get_msg_class_from_name, get_subscriptions
from fuzzer import flatten_nested_dict

import rclpy


def main(fuzz_type):
    rclpy.init()
    node = rclpy.create_node("replayer")
    time.sleep(1)

    if fuzz_type == "message":
        target_node = "/turtlesim"
        topic_name = "/turtle1/cmd_vel"
        msg_type_class = get_msg_class_from_name("geometry_msgs", "Twist")
        msg = msg_type_class()

        if len(sys.argv) > 1:
            msg_file = sys.argv[2]
        else:
            msg_file = "msgs/bug2-2.msg"

        msg_dict = pickle.load(open(msg_file, "rb"))

        l = flatten_nested_dict(msg_dict)
        for item in l:
            attr_list = item[:-1]
            attr_leaf = attr_list[-1]
            data_val = item[-1]

            obj = reduce(getattr, attr_list[:-1], msg)
            setattr(obj, attr_leaf, data_val)

        # msg.linear.x = 0.0
        # msg.linear.y = 0.0
        # msg.linear.z = 0.0
        # msg.angular.x = 0.0
        # msg.angular.y = 0.0
        # msg.angular.z = 375561095.7465013
        # msg.angular.z = 1.0
        print(msg)

        pub = node.create_publisher(msg_type_class, topic_name, 10)
        pub.publish(msg)

        time.sleep(1)

        subscriptions = get_subscriptions(node)
        assert target_node in subscriptions

    elif fuzz_type == "service":
        ldate = "20210219-180042"
        ltime = "1613775658.863716"

        meta_file = os.path.join("logs", ldate, "metadata", f"meta-{ltime}")
        with open(meta_file, "r") as fp:
            meta = fp.readline().split("\t")
        srv_name = meta[1]
        srv_type_full = meta[2]

        ss = srv_type_full.split("/")
        if len(ss) == 2:
            ss = [ss[0], "srv", ss[1]]
        srv_pkg = ss[0]
        srv_type = ss[-1]

        srv_module_name = '.'.join(ss[:-1])
        try:
            module = importlib.import_module(srv_module_name)
            srv_module = getattr(module, srv_type)
            if not srv_pkg or not srv_module:
                raise ValueError()
        except ValueError:
            raise RuntimeError("service type is invalid")

        client = node.create_client(srv_module, srv_name)
        request = srv_module.Request()

        req_file = os.path.join("logs", ldate, "queue", f"request-{ltime}")
        req_dict = pickle.load(open(req_file, "rb"))

        l = flatten_nested_dict(req_dict)
        for item in l:
            attr_list = item[:-1]
            attr_leaf = attr_list[-1]
            data_val = item[-1]

            obj = reduce(getattr, attr_list[:-1], request)
            setattr(obj, attr_leaf, data_val)

        print(request)
        future = client.call_async(request)

        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            print('response:\n%r\n' % future.result())
        else:
            raise RuntimeError('Exception while calling service: %r' % future.exception())

    rclpy.shutdown()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        exit(-1)
    fuzz_type = sys.argv[1].strip()
    if fuzz_type not in ["message", "service", "action"]:
        exit(-1)
    main(fuzz_type)
