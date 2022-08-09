import os
import pickle
import time

import rclpy
import ros_utils
from rosidl_runtime_py import message_to_ordereddict, set_message_fields

import config
from fuzzer import Fuzzer
from executor import Executor, ExecMode
from rosbag_parser import RosbagParser
import checker

def retrieve_message(frame, directory):

    found = False
    p_msg = ""
    for p_msg in os.listdir(os.path.join(directory, "queue")):
        if frame in p_msg:
            found = True
            break

    if not found:
        print("couldn't find msg")
        exit(-1)

    f = open(os.path.join(directory, "queue", p_msg), "rb")
    msg_dict = pickle.load(f)
    f.close()

    # print(msg_dict["data"])

    for meta in os.listdir(os.path.join(directory, "metadata")):
        if frame in meta:
            found = True
            break

    f = open(os.path.join(directory, "metadata", meta), "rb")
    meta_dump = f.readline()
    f.close()

    meta = meta_dump.split(b"\t")
    class_str = meta[2].decode("utf-8")

    # regen class
    type_name = class_str.split("'")[1].split(".")[-1]
    # type_name = "BoolUnboundedDynArray" # overriding it for now

    msg_type_class = ros_utils.get_msg_class_from_name(
            "idltest_msgs", type_name)
    msg = msg_type_class()

    if "Array" in type_name:
        for d in msg_dict["data"]:
            msg.data.append(d)
    else:
        msg.data = msg_dict["data"]

    print(msg)

    return (msg, msg_type_class, type_name)

if __name__ == "__main__":
    rclpy.init()

    frame = "1636629431.2263825"
    directory = "../logs/20211110-163031/"

    (msg, msg_type_class, type_name) = retrieve_message(frame, directory)
    topic_name = f"/idltest_{type_name}_in"
    msg_list = [msg]

    node = rclpy.create_node("replay_pub")
    pub = node.create_publisher(msg_type_class, topic_name, 10)
    print("publishing")
    pub.publish(msg)
    time.sleep(1)

    if False:
        config = config.RuntimeConfig()
        config.test_rosidl = True
        config.test_rosidl_lang = "cpp"
        config.watchlist = "watchlist/idltest.json"
        config.replay = True

        fuzzer = Fuzzer("_fuzzer", config)

        executor = Executor(fuzzer)
        executor.prep_execution(msg_type_class, topic_name)
        fuzzer.executor = executor

        pre_exec_list = None
        post_exec_list = None
        pub_function = None

        (retval, failure_msg) = executor.execute(
            mode=ExecMode.SINGLE,
            msg_list=msg_list,
            frame=frame,
            frequency=1,
            repeat=1,
            pre_exec_list=pre_exec_list,
            post_exec_list=post_exec_list,
            pub_function=pub_function,
            wait_lock=None,
        )
        executor.clear_execution()

        if retval:
            print("publish failed:", failure_msg)
        else:
            exec_cnt = 0
            parser = RosbagParser(
                f"states-{exec_cnt}.bag/states-{exec_cnt}.bag_0.db3"
            )

            state_msgs_dict = parser.process_messages()
            print("-----")
            print(state_msgs_dict)
            print("=====")

            errs = checker.run_checks(fuzzer.config, msg_list, state_msgs_dict)
            errs = list(set(errs))

        fuzzer.kill_target()
        fuzzer.destroy()

    rclpy.shutdown()
