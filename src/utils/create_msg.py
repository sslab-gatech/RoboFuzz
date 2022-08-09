#!/usr/bin/python3

from collections import OrderedDict
import pickle
import time

msg_file = "msgs/msg-1611007897.7347338" # turtle disappears
# msg_file = "msg-1611008929.8656843" # hits the wall
msg_dict = pickle.load(open(msg_file, "rb"))

print(msg_dict)

# geometry_msgs/Twist
msg_dict = OrderedDict()
d1 = {"x": 1.933767054481329e+241, "y": 0.0, "z": 0.0} # x
d1 = {"x": 1.933767054481329e+241, "y": 4.8355230813047836e-160, "z": 0.0} # x
d1 = {"x": 1.933767054481329e+241, "y": 4.8355230813047836e-160, "z": -3.958275365874432e+132}
d2 = {"x": -2.6477435485741285e-44, "y": -3.503376746735036e+183, "z": -5.474433982957781e+106}

d1 = {"x": 0.0, "y": 0.0, "z": 0.0}
d2 = {"x": 0.0, "y": 0.0, "z": -5.474433982957781e+106}
msg_dict["linear"] = OrderedDict(d1)
msg_dict["angular"] = OrderedDict(d2)

pickle.dump(msg_dict, open("msg-{}".format(time.time()), "wb"))

"""
OrderedDict([
    ('linear', OrderedDict([
        ('x', 1.933767054481329e+241),
         ('y', 4.8355230813047836e-160),
        ('z', -3.958275365874432e+132)
        ])
    ),
    ('angular', OrderedDict([
        ('x', -2.6477435485741285e-44),
        ('y', -3.503376746735036e+183),
        ('z', -5.474433982957781e+106)
        ])
    )
])
"""

