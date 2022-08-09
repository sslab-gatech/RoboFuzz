import struct
import sys
import numpy as np

sys.path.append("../")
import mutator as m


dtype = np.dtype("float32")
v = m.gen_rand_data(dtype)

print("original data")
print(v)
print(m.float32_to_bitstr(v))

for i in range(0, 32, 8):
    mutv = m.mutate_one(dtype, v, m.STAGE_ARITH8, i, 35)
    if mutv is not None:
        print(m.float32_to_bitstr(v), v)
        print("="*i + "v"*8 + "="*(24-i))
        print(m.float32_to_bitstr(mutv), mutv)
        print("")
    else:
        print("skip mutation")

for i in range(0, 32, 8):
    mutv = m.mutate_one(dtype, v, m.STAGE_ARITH16, i, 1)
    if mutv is not None:
        print(m.float32_to_bitstr(v), v)
        print("="*i + "v"*16 + "="*(16-i))
        print(m.float32_to_bitstr(mutv), mutv)
        print("")
    else:
        print("skip mutation")


dtype = np.dtype("float64")
v = m.gen_rand_data(dtype)

print("original data")
print(v)
print(m.float64_to_bitstr(v))

for i in range(0, 64, 8):
    mutv = m.mutate_one(dtype, v, m.STAGE_ARITH32, i, 35)
    if mutv is not None:
        print(m.float64_to_bitstr(v), v)
        print("="*i + "v"*32 + "="*(32-i))
        print(m.float64_to_bitstr(mutv), mutv)
        print("")
    else:
        print("skip mutation")

