import time
import os

while True:
    t = str(time.time())
    cmd = "ros2 topic pub --once /sros2_input std_msgs/String '{data: " + t + "}'"
    os.system(cmd)
