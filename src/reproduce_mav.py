import os
import re
import pickle
import time
import json
import signal
import subprocess as sp
import shutil

import px4_utils
import checker
from rosbag_parser import RosbagParser


class Msg:
    """a dummy message class for compatibility with px4_utils"""
    def __init__(self, x, y, z, r):
        self.x = x
        self.y = y
        self.z = z
        self.r = r

class Config:
    """a dummy config class for compatibility with checker"""
    px4_sitl = True
    rospkg = None
    tb3_sitl = False
    tb3_hitl = False
    test_rosidl = False


def get_subframe(filename):
    return filename.split("-")[-1]


def start_rosbag(frame):
    watchlist_file = "watchlist/px4.json"
    bag_dir = f"states-replay-{frame}.bag"
    print("bag:", bag_dir)

    with open(watchlist_file, "r") as fp:
        data = fp.read()

    try:
        shutil.rmtree(bag_dir)
    except:
        print("[executor] could not remove previous bag dir")

    rosbag_cmd = f"ros2 bag record -o {bag_dir} --qos-profile-overrides-path qos_override.yaml "
    watchlist = json.loads(data)
    for target in watchlist:
        rosbag_cmd += target + " "
    rosbag_cmd += "listen_flag"

    rosbag_pgrp = sp.Popen(
        rosbag_cmd,
        shell=True,
        preexec_fn=os.setpgrp,
        stdout=sp.PIPE,
        stderr=sp.PIPE,
    )
    print("[executor] started ros2 bag recording")
    # print(rosbag_cmd)

    # need time for topics to be discovered
    time.sleep(1)

    return rosbag_pgrp


def start_watching():
    print("start watching", time.time())

    sp.call(
        [
            "ros2",
            "topic",
            "pub",
            "--once",
            "/listen_flag",
            "std_msgs/UInt64",
            "{data: " + str(int(str(time.time()).replace(".", ""))) + "}",
        ],
        stdout=sp.DEVNULL,
        stderr=sp.DEVNULL,
    )

def stop_watching():
    print("stop watching", time.time())

    sp.call(
        [
            "ros2",
            "topic",
            "pub",
            "--once",
            "/listen_flag",
            "std_msgs/UInt64",
            "{data: " + str(int(str(time.time()).replace(".", ""))) + "}",
        ],
        stdout=sp.DEVNULL,
        stderr=sp.DEVNULL,
    )


log_dir = "logs/20220104-204610/"
frame = "1641356645.0672185"

log_dir = "logs/20220105-165934/"
frame = "1641419985.117267"

# interesting
log_dir = "logs/20220105-193319/"
frame = "1641444258.535236"
log_dir = "logs/20220105-193319/"
# frame = "1641443932.2358694"
frame = "1641441647.26109"
log_dir = "logs/20220128-122952"
frame = "1643407191.4075294"

msg_file_list = [
    f
    for f in os.listdir(os.path.join(log_dir, "queue"))
    if re.match(r'msg-' + frame + r'-*', f)
]

msg_file_list.sort(key=get_subframe)
msg_list = list()

for msg in msg_file_list:
    msg_dict = pickle.load(open(os.path.join(log_dir, "queue", msg), "rb"))
    msg_dict = dict(msg_dict)
    msg = Msg(msg_dict["x"], msg_dict["y"], msg_dict["z"], msg_dict["r"])
    msg_list.append(msg)

px4_bridge = px4_utils.Px4BridgeNode()
px4_bridge.init_mavlink()
px4_bridge.prepare_flight()

rosbag_pgrp = start_rosbag(frame)
start_watching()

for msg in msg_list:
    # print(msg.x, msg.y, msg.z, msg.r)
    px4_bridge.send_command(msg)
    time.sleep(0.1)

stop_watching()
os.killpg(rosbag_pgrp.pid, signal.SIGINT)


parser = RosbagParser(
    f"states-replay-{frame}.bag/states-replay-{frame}.bag_0.db3"
)
state_msgs_dict = parser.process_messages()

config = Config()

errs = checker.run_checks(config, msg_list, state_msgs_dict)
print(errs)


