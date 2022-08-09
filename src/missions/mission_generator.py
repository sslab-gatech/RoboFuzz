import json

from rosidl_runtime_py import message_to_ordereddict, set_message_fields
from px4_msgs.msg import TrajectorySetpoint


# missionfile = open("church-fail.json", "w")
missionfile = open("takeoff-quickland.json", "w")

msg_list = []

# takeoff
msg = TrajectorySetpoint()
msg.z = -10.0
msg.yaw = 0.0
# msg.acceleration[2] = -3000.0

md = message_to_ordereddict(msg)
for i in range(50):
    msg_list.append(md)

# quick landing
msg = TrajectorySetpoint()
msg.z = 0.0
msg.yaw = 0.0
msg.acceleration[2] = 3000.0

md = message_to_ordereddict(msg)
for i in range(50):
    msg_list.append(md)

# msg = TrajectorySetpoint()
# msg.x = 20.0
# msg.y = 10.0
# msg.z = -5.0
# msg.yaw = 1.0
# msg.acceleration[1] = 3000.0

# md_f = message_to_ordereddict(msg)

# # waypoint 1
# msg = TrajectorySetpoint()
# msg.x = 20.0
# msg.y = 10.0
# msg.z = -5.0
# msg.yaw = 1.0

# md = message_to_ordereddict(msg)
# for i in range(50):
    # msg_list.append(md)

    # # inject failure
    # # if i == 20:
        # # msg_list.append(md_f)

# # waypoint 2
# msg = TrajectorySetpoint()
# msg.x = 35.0
# msg.y = 10.0
# msg.z = -5.0
# msg.yaw = -0.2

# md = message_to_ordereddict(msg)
# for i in range(50):
    # msg_list.append(md)

# # waypoint 3
# msg = TrajectorySetpoint()
# msg.x = 25.0
# msg.y = -10.0
# msg.z = -5.0
# msg.yaw = -1.8

# md = message_to_ordereddict(msg)
# for i in range(50):
    # msg_list.append(md)

# # waypoint 4 (return)
# msg = TrajectorySetpoint()
# msg.x = 0.0
# msg.y = 0.0
# msg.z = -5.0
# msg.yaw = 3.0

# md = message_to_ordereddict(msg)
# for i in range(50):
    # msg_list.append(md)

# # landing
# msg = TrajectorySetpoint()
# msg.x = 0.0
# msg.y = 0.0
# msg.z = -0.0
# msg.yaw = 0.0

# md = message_to_ordereddict(msg)
# for i in range(100):
    # msg_list.append(md)

json.dump(msg_list, missionfile)
missionfile.close()
