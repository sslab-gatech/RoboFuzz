import struct
import sys
import numpy as np

sys.path.append("../")
import mutator as m


dtype = np.dtype("int16")
v = m.gen_rand_data(dtype)

print("original data")
print(v)
print(m.int16_to_bitstr(v))

for i in range(0, 16, 8):
    mutv = m.mutate_one(dtype, v, m.STAGE_ARITH8, i, 35)
    if mutv is not None:
        print(m.int16_to_bitstr(v), v)
        print(m.int16_to_bitstr(mutv), mutv)
        print("")

for i in range(0, 16, 8):
    mutv = m.mutate_one(dtype, v, m.STAGE_ARITH16, i, 1)
    if mutv is not None:
        print(m.int16_to_bitstr(v), v)
        print(m.int16_to_bitstr(mutv), mutv)
        print("")

dtype = np.dtype("int32")
v = m.gen_rand_data(dtype)

print("original data")
print(v)
print(m.int_to_bitstr(32, v))

for i in range(0, 32, 8):
    mutv = m.mutate_one(dtype, v, m.STAGE_ARITH8, i, 35)
    if mutv is not None:
        print(m.int32_to_bitstr(v), v)
        print("="*i + "v"*8 + "="*(24-i))
        print(m.int32_to_bitstr(mutv), mutv)
        print("")
    else:
        print("skip mutation")

for i in range(0, 32, 8):
    mutv = m.mutate_one(dtype, v, m.STAGE_ARITH16, i, 1)
    if mutv is not None:
        print(m.int32_to_bitstr(v), v)
        print("="*i + "v"*16 + "="*(16-i))
        print(m.int32_to_bitstr(mutv), mutv)
        print("")
    else:
        print("skip mutation")


dtype = np.dtype("int64")
v = m.gen_rand_data(dtype)

print("original data")
print(v)
print(m.int_to_bitstr(64, v))

for i in range(0, 64, 8):
    mutv = m.mutate_one(dtype, v, m.STAGE_ARITH32, i, 35)
    if mutv is not None:
        print(m.int64_to_bitstr(v), v)
        print("="*i + "v"*32 + "="*(32-i))
        print(m.int64_to_bitstr(mutv), mutv)
        print("")
    else:
        print("skip mutation")

# for i in range(32):
    # mutv = mutator.mutate_one(dtype, v, mutator.STAGE_FLIP8, i)
    # print("="*(i) + "^"*8 + "="*(32-i-8))
    # print(mutator.float32_to_bitstr(v))
    # print(mutator.float32_to_bitstr(mutv), mutv)
    # print("")


# v = mutator.rand_float64()
# print(v)
# print(mutator.float64_to_bitstr(v))

# dtype = numpy.dtype("float64")
# for i in range(64):
    # mutv = mutator.mutate_one(dtype, v, mutator.STAGE_FLIP1, i)
    # print("="*(i) + "^" + "="*(64-i-1))
    # print(mutator.float64_to_bitstr(v))
    # print(mutator.float64_to_bitstr(mutv), mutv)
    # print("")

# for i in range(64):
    # mutv = mutator.mutate_one(dtype, v, mutator.STAGE_FLIP2, i)
    # print("="*(i) + "^"*2 + "="*(64-i-2))
    # print(mutator.float64_to_bitstr(v))
    # print(mutator.float64_to_bitstr(mutv), mutv)
    # print("")
