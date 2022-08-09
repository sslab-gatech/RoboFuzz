import json

from rosidl_runtime_py import message_to_ordereddict, set_message_fields
from px4_msgs.msg import TrajectorySetpoint

# missionfile = open("church-fail.json", "w")
missionfile = open("takeoff-push.json", "w")

msg_list = []

# takeoff
msg = TrajectorySetpoint()
msg.z = -10.0
msg.yaw = 0.0
msg.acceleration[2] = -3000.0

md = message_to_ordereddict(msg)
for i in range(50):
    msg_list.append(md)

json.dump(msg_list, missionfile)
missionfile.close()
