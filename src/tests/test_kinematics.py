import math

import kinpy as kp
"""
prerequisite:
$ pip3 install kinpy
"""

panda_urdf = "/opt/ros/foxy/share/moveit_resources_panda_description/urdf/panda.urdf"
chain = kp.build_chain_from_urdf(open(panda_urdf).read())
serial_chain = kp.build_serial_chain_from_urdf(
    open(panda_urdf).read(),
    "panda_hand"
)

# print(chain)
# print(chain.get_joint_parameter_names())

# print(serial_chain)
# print(serial_chain.get_joint_parameter_names())


# default position
th_default = {
    "panda_finger_joint1": math.radians(0),
    "panda_joint1": math.radians(0),
    "panda_joint2": math.radians(-45),
    "panda_finger_joint2": math.radians(0),
    "panda_joint3": math.radians(0),
    "panda_joint4": math.radians(-135),
    "panda_joint5": math.radians(0),
    "panda_joint6": math.radians(90),
    "panda_joint7": math.radians(45),
}

# x:=0.28 y:=-0.2 z:=0.3 w:=1.0
"""
$ ros2 topic echo /joint_states
...
name:
- panda_finger_joint1
- panda_joint1
- panda_joint2
- panda_finger_joint2
- panda_joint3
- panda_joint4
- panda_joint5
- panda_joint6
- panda_joint7
position:
- 0.0
- -1.897337603173183
- -1.7623374174942728
- 0.0
- 2.0174633194485563
- -2.6834497838874456
- -1.0873036202046826
- 1.5543641496734297
- 1.487687515257222
"""
th = {
    "panda_finger_joint1": 0.0,
    "panda_joint1": -1.897337603173183,
    "panda_joint2": -1.7623374174942728,
    "panda_finger_joint2": 0.0,
    "panda_joint3": 2.0174633194485563,
    "panda_joint4": -2.6834497838874456,
    "panda_joint5": -1.0873036202046826,
    "panda_joint6": 1.5543641496734297,
    "panda_joint7": 1.487687515257222
}

"""
- 0.0
- 3.761173896491529e-05
- 5.496248295528061e-05
- 0.0
- 7.947159116156401e-05
- 5.475179431881512e-06
- -4.8680176725611096e-05
- 1.5709897997769993
- 0.7850355813402683
"""
th_extended = {
    "panda_finger_joint1": 0.0,
    "panda_joint1": 3.761173896491529e-05,
    "panda_joint2": 5.496248295528061e-05,
    "panda_finger_joint2": 0.0,
    "panda_joint3": 7.947159116156401e-05,
    "panda_joint4": 5.475179431881512e-06,
    "panda_joint5": -4.8680176725611096e-05,
    "panda_joint6": 1.5709897997769993,
    "panda_joint7": 0.7850355813402683
}


fk = chain.forward_kinematics(th_default)
print(fk["panda_hand"])
eff_pos = fk["panda_hand"].pos
# print(eff_pos[0], eff_pos[1], eff_pos[2])
"""
Transform(
    rot=[ 9.24016073e-01  2.43041390e-04 -5.21072594e-05 -3.82353548e-01],
    pos=[ 0.28003789 -0.20003787  0.30004261]
)
"""
import numpy as np
import copy
tf_nosol = kp.Transform()
tf_nosol.rot[0] = 0.0
tf_nosol.rot[1] = 0.0
tf_nosol.rot[2] = 0.0
tf_nosol.rot[3] = 1.0
tf_nosol.pos[0] = 20.0
tf_nosol.pos[1] = 0.0
tf_nosol.pos[2] = 0.0
print(tf_nosol)

import math
ik = serial_chain.inverse_kinematics(tf_nosol)
print(serial_chain.get_joint_parameter_names())
for joint_angle in ik:
    print(math.degrees(joint_angle))

