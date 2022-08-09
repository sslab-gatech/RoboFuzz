import struct
import sys
import numpy as np

sys.path.append("../")
import mutator as m


stage = m.STAGE_FLIP1
dtype = np.dtype("int8")
data_val = 10
for i in range(8):
    mutated = m.mutate_one(dtype, data_val, stage, i)
    print(mutated, bin(mutated))

stage = m.STAGE_FLIP2
dtype = np.dtype("int8")
data_val = 10
for i in range(8):
    mutated = m.mutate_one(dtype, data_val, stage, i)
    print(mutated, bin(mutated))

stage = m.STAGE_FLIP4
dtype = np.dtype("int8")
data_val = 10
for i in range(8):
    mutated = m.mutate_one(dtype, data_val, stage, i)
    print(mutated, bin(mutated))

stage = m.STAGE_FLIP8
dtype = np.dtype("int8")
data_val = 10
for i in range(8):
    mutated = m.mutate_one(dtype, data_val, stage, i)
    print(mutated, bin(mutated))
