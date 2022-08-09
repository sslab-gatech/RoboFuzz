import sys
import os
import importlib
import time
import pickle
import json
import threading
import logging

import constants as c

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_srvs.srv import Empty


import ros_utils


class StateMonitorNode(Node):
    msg_queue = list()
    subs = dict()
    do_record = False
    is_recording = False

    def __init__(self, watchlist):
        super().__init__("_state_monitor")

        self.watchlist = watchlist

        # self.srv_start_monitor = self.create_service(Empty, "start_monitor",
        #     self.start_monitor_callback)

        # self.srv_stop_monitor = self.create_service(Empty, "stop_monitor",
        #     self.stop_monitor_callback)

        self.logger = self.init_logger()

        # self.listen_flag = self.create_subscription(
        #     Bool,
        #     "_listen_flag",
        #     self.listen_callback,
        #     10
        # )

        # re-create subscriptions every time monitoring is requested
        for target in self.watchlist:
            topic_msg_type = self.watchlist[target]
            pkg_name = ".".join(topic_msg_type.split("/")[:-1])
            module_name = topic_msg_type.split("/")[-1]

            msg_type_class = ros_utils.get_msg_class_from_name(
                pkg_name.split(".")[0], module_name)

            self.subs[target] = self.create_subscription(
                    msg_type_class, target, self.msg_callback, 10)
            print(f"[state monitor] subscribed to {target}")
            self.logger.debug(f"subscribed to {target}")
    
    def init_logger(self):
        logger = logging.getLogger("state_monitor")
        logger.setLevel(logging.DEBUG)

        fh = logging.FileHandler("state_monitor.log")
        fh.setLevel(logging.DEBUG)
        fmt = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        fh.setFormatter(fmt)

        logger.addHandler(fh)

        return logger
    
    # def start_monitor_callback(self, request, response):
    #     print("[state monitor] start_monitor service called")
    #     self.logger.debug("start_monitor service called")

    #     self.do_record = True

    #     self.logger.debug(f"returning from start_monitor_callback")
    #     return response

    # def stop_monitor_callback(self, request, response):
    #     print("[state monitor] stop_monitor service called")
    #     self.logger.debug("stop_monitor service called")
    #     self.do_record = False

    #     # print(len(self.msg_queue))

    #     # self.is_recording = False
    #     self.dump_to_file()

    #     self.msg_queue = list()

    #     # # destroy subscriptions
    #     # for target in self.subs:
    #     #     sub = self.subs[target]
    #     #     print(sub)
    #     #     sub.destroy()
    #     #     print(f"[state monitor] unsubscribed from {target}")
    #     # self.subs = dict()

    #     # # destroy subscriptions
    #     # for target in self.subs:
    #     #     res = self.destroy_subscription(self.subs[target])
    #     #     assert(res is True)
    #     #     print(f"[state monitor] unsubscribed from {target}")
    #     #     self.logger.debug(f"unsubscribed from {target}")

    #     try:
    #         os.remove(c.PUBLOCK)
    #     except:
    #         pass

    #     print("[state monitor] stop_monitor_callback returning")
    #     self.logger.debug("stop_monitor_callback returning")
    #     return response

    def msg_callback(self, msg):
        # These callbacks are not being called persistently as expected;
        # when start_monitor request is received, the node is supposed
        # to be already listening to the watchlist topic, and calling
        # this callback. However, this callback function is called a while
        # after the request is handled.
        # Tried many different things to fix this, but nothing works.
        # Will move on by just stalling the executor from publishing messages
        # until this callback is called by placing a barrier
        # so that no state is lost after messages are published :(

        self.logger.debug("callback", self.is_recording)
        if not os.path.isfile(c.PUBLOCK):
            self.logger.debug("creating publock file")
            fp = open(c.PUBLOCK, "w")
            fp.close()


        if not os.path.isfile(c.WATCHFLAG):
            if self.is_recording:
                self.is_recording = False
                self.logger.debug("dumping states to file")
                self.dump_to_file()
                self.msg_queue = list()

                try:
                    os.remove(c.PUBLOCK)
                except:
                    pass

        else:
            self.is_recording = True

            self.logger.debug("enqueueing msgs")
            self.msg_queue.append(msg)

        # if self.do_record:
        #     self.msg_queue.append(msg)

    def listen_callback(self, msg):
        if msg.data is True:
            self.do_record = True
            self.is_recording = True
            print("start recording")
        else:
            self.do_record = False
            if self.is_recording:
                print("stop recording")
                print(len(self.msg_queue))
                self.is_recording = False
                self.dump_to_file()
                self.msg_queue = list()

    def dump_to_file(self):
        pickle.dump(self.msg_queue, open("states.pkl", "wb"))


def main(watchlist_file):
    rclpy.init()

    with open(watchlist_file, "r") as fp:
        data = fp.read()

    watchlist = json.loads(data)

    _sm = StateMonitorNode(watchlist)
    node_executor = rclpy.executors.MultiThreadedExecutor(12)
    node_executor.add_node(_sm)
    node_executor_thread = threading.Thread(
        target=node_executor.spin,
        daemon=True
    )
    # rclpy.spin(_sm)
    node_executor_thread.start()

    rate = _sm.create_rate(10) # 2 Hz
    try:
        while rclpy.ok():
            # print("[state monitor] listening, {}".format(_sm.do_record))
            rate.sleep()
    except KeyboardInterrupt:
        print("[state monitor] aborting as user requested")

    _sm.destroy_node()
    rclpy.shutdown()
    node_executor_thread.join()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: $ python3 {} [watchlist_file]".format(sys.argv[0]))
        sys.exit(1)

    watchlist_file = sys.argv[1].strip()
    if not os.path.isfile(watchlist_file):
        print("Cannot find {}".format(watchlist_file))
        sys.exit(1)

    try:
        os.remove("states.pkl")
    except:
        pass

    try:
        os.remove(c.PUBLOCK)
    except:
        pass

    try:
        os.remove(c.WATCHFLAG)
    except:
        pass

    main(watchlist_file)

