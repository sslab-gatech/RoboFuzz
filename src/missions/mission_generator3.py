import json

from rosidl_runtime_py import message_to_ordereddict, set_message_fields
from px4_msgs.msg import TrajectorySetpoint

missionfile = open("takeoff-pushdown.json", "w")

msg_list = []

# normal takeoff to z = 20
msg = TrajectorySetpoint()
msg.z = -30.0
msg.yaw = 0.0
# msg.acceleration[2] = -3000.0

md = message_to_ordereddict(msg)
for i in range(100):
    msg_list.append(md)

# pushdown to z = 10
msg = TrajectorySetpoint()
msg.z = 0.0
msg.yaw = 0.0
msg.acceleration[2] = 5000.0

md = message_to_ordereddict(msg)
for i in range(5):
    msg_list.append(md)

json.dump(msg_list, missionfile)
missionfile.close()
