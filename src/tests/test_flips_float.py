import struct
import sys
import numpy as np

sys.path.append("../")
import mutator as m


v = m.rand_float32()

print(v)
print(m.float32_to_bitstr(v))

dtype = np.dtype("float32")
for i in range(32):
    mutv = m.mutate_one(dtype, v, m.STAGE_FLIP1, i)
    print("="*(i) + "^" + "="*(32-i-1))
    print(m.float32_to_bitstr(v))
    print(m.float32_to_bitstr(mutv), mutv)
    print("")

for i in range(32):
    mutv = m.mutate_one(dtype, v, m.STAGE_FLIP8, i)
    print("="*(i) + "^"*8 + "="*(32-i-8))
    print(m.float32_to_bitstr(v))
    print(m.float32_to_bitstr(mutv), mutv)
    print("")


v = m.rand_float64()
print(v)
print(m.float64_to_bitstr(v))

dtype = np.dtype("float64")
for i in range(64):
    mutv = m.mutate_one(dtype, v, m.STAGE_FLIP1, i)
    print("="*(i) + "^" + "="*(64-i-1))
    print(m.float64_to_bitstr(v))
    print(m.float64_to_bitstr(mutv), mutv)
    print("")

for i in range(64):
    mutv = m.mutate_one(dtype, v, m.STAGE_FLIP2, i)
    print("="*(i) + "^"*2 + "="*(64-i-2))
    print(m.float64_to_bitstr(v))
    print(m.float64_to_bitstr(mutv), mutv)
    print("")
