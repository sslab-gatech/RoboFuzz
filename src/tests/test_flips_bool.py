import struct
import sys
import numpy as np

sys.path.append("../")
import mutator as m


stage = m.STAGE_FLIP1
dtype = np.dtype("bool")
data_val = True
for i in range(8):
    mutated = m.mutate_one(dtype, data_val, stage, i)
    if mutated is not None:
        print(mutated, bin(mutated))

stage = m.STAGE_FLIP2
for i in range(8):
    mutated = m.mutate_one(dtype, data_val, stage, i)
    if mutated is not None:
        print(mutated, bin(mutated))

