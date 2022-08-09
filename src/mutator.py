import os
import numpy as np
import struct
import math
import random
import string
import unicodedata

import constants as c

DEBUG = 0

BMP_MIN = int("0X0000", 16)
BMP_MAX = int("0XFFFF", 16)
SMP_MIN = int("0X10000", 16)
SMP_MAX = int("0X1FFFF", 16)

UNICODE_LETTERS = []
for i in range(BMP_MIN, SMP_MAX):
    char = chr(i)
    if unicodedata.category(char).startswith("L"):
        UNICODE_LETTERS.append(char)


def rand_float32():
    bitstr = str(np.random.randint(2)) # sign
    for i in range(8): # exponent
        bitstr += str(np.random.randint(2))
    for i in range(23): # fraction
        bitstr += str(np.random.randint(2))

    val_int = int(bitstr, 2)
    val_bytes = val_int.to_bytes(4, "big")
    val = struct.unpack(">f", val_bytes)[0]

    if math.isnan(val):
        return rand_float64()

    return val


def rand_float64():
    bitstr = str(np.random.randint(2)) # sign
    for i in range(11): # exponent
        bitstr += str(np.random.randint(2))
    for i in range(52): # fraction
        bitstr += str(np.random.randint(2))

    val_int = int(bitstr, 2)
    val_bytes = val_int.to_bytes(8, "big")
    val = struct.unpack(">d", val_bytes)[0]

    if math.isnan(val):
        return rand_float64()

    return val


def rand_unicode(length):
    return "".join([random.choice(UNICODE_LETTERS) for i in range(length)])


def gen_rand_data(dtype, default=False):

    if hasattr(dtype, "type"):
        if dtype.type is np.bool_:
            # print("bool")
            if default:
                val = True
            else:
                val = True if np.random.randint(2) else False

        elif dtype.type is np.byte:
            # print("byte")
            if default:
                val = b"\x00"
            else:
                val = os.urandom(1)

        elif dtype.type is np.char:
            # print("char")
            if default:
                val = "\0"
            else:
                val = str(os.urandom(1))

        elif dtype.type is np.str_:
            str_len_max = c.STRLEN_MAX
            str_len = np.random.randint(str_len_max)

            if default:
                val = "\n"
            else:
                # val = str(os.urandom(np.random.randint(str_len_max)).decode("latin1"))
                val = rand_unicode(str_len)

        elif np.issubdtype(dtype, np.integer):
            # print("int")
            if default:
                val = 0
            else:
                val = int(np.random.randint(np.iinfo(dtype).min,
                        np.iinfo(dtype).max+1, dtype=dtype.type))

        elif np.issubdtype(dtype, np.floating):
            if default:
                val = 0.0
            else:
                if dtype.type is np.float32:
                    # print("float32")
                    val = rand_float32()
                elif dtype.type is np.float64:
                    # print("float64")
                    val = rand_float64()
        else:
            # print("UNSUPPORTED TYPE:", dtype)
            val = None

    else:
        # print("complex type")
        val = [0.0, 0.0, 0.0]

    return val


def gen_int_in_range(low, high):
    val = np.random.randint(low=low, high=high+1)

    return val


def gen_float_in_range(low, high, precision=6):
    """
    Generate a float value in small (~1000) range and limited
    precision.
    """
    int_part = gen_int_in_range(low, high)
    dec_part = gen_int_in_range(0, pow(10, precision))

    return int_part + float(dec_part) / pow(10, precision)


def gen_special_floats():
    i = np.random.randint(0, 3)

    if i == 0:
        return float("inf")

    elif i == 1:
        return float("-inf")

    elif i == 2:
        return float("nan")


def int_to_bitstr(size, value):
    if size == 8:
        return int8_to_bitstr(value)
    elif size == 16:
        return int16_to_bitstr(value)
    elif size == 32:
        return int32_to_bitstr(value)
    elif size == 64:
        return int64_to_bitstr(value)


def int8_to_bitstr(value):
    if value < 0:
        return bin(value & 0xff)[2:]
    else:
        return format(value, "08b")


def int16_to_bitstr(value):
    if value < 0:
        return bin(value & 0xffff)[2:]
    else:
        return format(value, "016b")


def int32_to_bitstr(value):
    if value < 0:
        return bin(value & 0xffffffff)[2:]
    else:
        return format(value, "032b")


def int64_to_bitstr(value):
    if value < 0:
        return bin(value & 0xffffffffffffffff)[2:]
    else:
        return format(value, "064b")


def float32_to_bitstr(value):
    return format(struct.unpack('!I', struct.pack('!f', value))[0], '032b')


def float64_to_bitstr(value):
    return format(struct.unpack('!Q', struct.pack('!d', value))[0], '064b')


def str_to_bitstr(value):
    value_bitstr = ""
    for char in value:
        c_int = ord(char)
        c_bitstr = int8_to_bitstr(c_int)
        value_bitstr += c_bitstr

    if len(value_bitstr) < c.STRLEN_MAX * 8:
        value_bitstr = "0" * (c.STRLEN_MAX * 8 - len(value_bitstr)) + value_bitstr

    return value_bitstr


# AFL's mutation stages
STAGE_FLIP1 = 0
STAGE_FLIP2 = 1
STAGE_FLIP4 = 2
STAGE_FLIP8 = 3
STAGE_FLIP16 = 4
STAGE_FLIP32 = 5
STAGE_ARITH8 = 6
STAGE_ARITH16 = 7
STAGE_ARITH32 = 8
STAGE_INTEREST8 = 9
STAGE_INTEREST16 = 10
STAGE_INTEREST32 = 11
STAGE_EXTRAS_UO = 12
STAGE_EXTRAS_UI = 13
STAGE_EXTRAS_AO = 14
STAGE_HAVOC = 15
STAGE_SPLICE = 16
STAGE_RANDOM = 17

STAGE_NAMES = {
    STAGE_FLIP1: "flip 1/1",
    STAGE_FLIP2: "flip 2/1",
    STAGE_FLIP4: "flip 4/1",
    STAGE_FLIP8: "flip 8/8",
    STAGE_FLIP16: "flip 16/8",
    STAGE_FLIP32: "flip 32/8",
    STAGE_ARITH8: "arith 8/8",
    STAGE_ARITH16: "arith 16/8",
    STAGE_ARITH32: "arith 32/8",
    STAGE_INTEREST8: "interest 8/8",
    STAGE_INTEREST16: "interest 16/8",
    STAGE_INTEREST32: "interest 32/8",
    STAGE_EXTRAS_UO: "extra user overwrite",
    STAGE_EXTRAS_UI: "extra insert",
    STAGE_EXTRAS_AO: "extra auto overwrite",
    STAGE_HAVOC: "havoc",
    STAGE_SPLICE: "splice",
    STAGE_RANDOM: "random"
}

MUT_WINDOW_SIZE = {
    STAGE_FLIP1: 1,
    STAGE_FLIP2: 2,
    STAGE_FLIP4: 4,
    STAGE_FLIP8: 8,
    STAGE_FLIP16: 16,
    STAGE_FLIP32: 32,
    STAGE_ARITH8: 8,
    STAGE_ARITH16: 16,
    STAGE_ARITH32: 32,
    STAGE_INTEREST8: 8,
    STAGE_INTEREST16: 16,
    STAGE_INTEREST32: 32,
}

APPLICABLE_STAGES = {
    "bool": [
        STAGE_FLIP1
    ],
    "byte": [
        STAGE_FLIP1, STAGE_FLIP2, STAGE_FLIP4, STAGE_FLIP8,
        STAGE_ARITH8,
        STAGE_INTEREST8
    ],
    "char": [
        STAGE_FLIP1, STAGE_FLIP2, STAGE_FLIP4, STAGE_FLIP8,
        STAGE_ARITH8,
        STAGE_INTEREST8
    ],
    "str": [
        STAGE_RANDOM
    ],
    "int8": [
        STAGE_FLIP1, STAGE_FLIP2, STAGE_FLIP4, STAGE_FLIP8,
        STAGE_ARITH8,
        STAGE_INTEREST8
    ],
    "uint8": [
        STAGE_FLIP1, STAGE_FLIP2, STAGE_FLIP4, STAGE_FLIP8,
        STAGE_ARITH8,
        STAGE_INTEREST8
    ],
    "int16": [
        STAGE_FLIP1, STAGE_FLIP2, STAGE_FLIP4, STAGE_FLIP8, STAGE_FLIP16,
        STAGE_ARITH8, STAGE_ARITH16,
        STAGE_INTEREST8, STAGE_INTEREST16
    ],
    "uint16": [
        STAGE_FLIP1, STAGE_FLIP2, STAGE_FLIP4, STAGE_FLIP8, STAGE_FLIP16,
        STAGE_ARITH8, STAGE_ARITH16,
        STAGE_INTEREST8, STAGE_INTEREST16
    ],
    "int32": [
        STAGE_FLIP1, STAGE_FLIP2, STAGE_FLIP4, STAGE_FLIP8, STAGE_FLIP16,
        STAGE_FLIP32,
        STAGE_ARITH8, STAGE_ARITH16, STAGE_ARITH32,
        STAGE_INTEREST8, STAGE_INTEREST16, STAGE_INTEREST32
    ],
    "uint32": [
        STAGE_FLIP1, STAGE_FLIP2, STAGE_FLIP4, STAGE_FLIP8, STAGE_FLIP16,
        STAGE_FLIP32,
        STAGE_ARITH8, STAGE_ARITH16, STAGE_ARITH32,
        STAGE_INTEREST8, STAGE_INTEREST16, STAGE_INTEREST32
    ],
    "int64": [
        STAGE_FLIP1, STAGE_FLIP2, STAGE_FLIP4, STAGE_FLIP8, STAGE_FLIP16,
        STAGE_FLIP32,
        STAGE_ARITH8, STAGE_ARITH16, STAGE_ARITH32,
        STAGE_INTEREST8, STAGE_INTEREST16, STAGE_INTEREST32
    ],
    "uint64": [
        STAGE_FLIP1, STAGE_FLIP2, STAGE_FLIP4, STAGE_FLIP8, STAGE_FLIP16,
        STAGE_FLIP32,
        STAGE_ARITH8, STAGE_ARITH16, STAGE_ARITH32,
        STAGE_INTEREST8, STAGE_INTEREST16, STAGE_INTEREST32
    ],
    "float32": [
        STAGE_FLIP1, STAGE_FLIP2, STAGE_FLIP4, STAGE_FLIP8, STAGE_FLIP16,
        STAGE_FLIP32,
        STAGE_ARITH8, STAGE_ARITH16, STAGE_ARITH32,
        STAGE_INTEREST8, STAGE_INTEREST16, STAGE_INTEREST32
    ],
    "float64": [
        STAGE_FLIP1, STAGE_FLIP2, STAGE_FLIP4, STAGE_FLIP8, STAGE_FLIP16,
        STAGE_FLIP32,
        STAGE_ARITH8, STAGE_ARITH16, STAGE_ARITH32,
        STAGE_INTEREST8, STAGE_INTEREST16, STAGE_INTEREST32
    ],
}


def bitlist_to_binary(bitlist):
    out = 0
    for bit in bitlist:
        out = (out << 1) | bit

    return out


BIT_LEN = {
    np.dtype("bool"): 1,
    np.dtype("byte"): 8,
    np.dtype("int8"): 8,
    np.dtype("uint8"): 8,
    np.dtype("int16"): 16,
    np.dtype("uint16"): 16,
    np.dtype("int32"): 32,
    np.dtype("uint32"): 32,
    np.dtype("int64"): 64,
    np.dtype("uint64"): 64,
    np.dtype("float32"): 32,
    np.dtype("float64"): 64,
    np.dtype("str_"): c.STRLEN_MAX * 8
}

INTERESTING_8 = [
    -128,
    -1,
    0,
    1,
    16,
    32,
    64,
    100,
    127
]

INTERESTING_16 = [
    -32768,
    -129,
    128,
    255,
    256,
    512,
    1000,
    1024,
    4096,
    32767
]

INTERESTING_32 = [
    -2147483648,
    -100663046,
    -32769,
    32768,
    65535,
    65536,
    100663045,
    2147483647
]

INTERESTING_MAP = {
    STAGE_INTEREST8: INTERESTING_8,
    STAGE_INTEREST16: INTERESTING_16,
    STAGE_INTEREST32: INTERESTING_32,
}

def get_primary_type(full_type_name):
    name = full_type_name.lower()
    if "bool" in name:
        return "bool"
    elif "byte" in name:
        return "byte"
    elif "char" in name:
        return "char"
    elif "int" in name:
        return "int"
    elif "float" in name:
        return "float"
    elif "string" in name:
        return "string"
    else:
        print("[-] Invalid type name")
        exit(-1)

def random_builtin_type_except(type_except):
    """Get a random type that differs from the given primary type"""
    primary_type = get_primary_type(type_except)

    compatible_types = []
    if primary_type == "char":
        compatible_types = ["int", "bool"]
    elif primary_type == "int":
        compatible_types = ["char", "bool"]

    while True:
        ros_type = random.choice(list(c.BuiltInType))
        sel_type_name_lower = ros_type.name.lower()

        # if the selected type is compatible with the given type,
        # don't break
        if primary_type not in sel_type_name_lower:
            break_flag = True
            for ct in compatible_types:
                if ct in sel_type_name_lower:
                    break_flag = False

            if break_flag:
                break

    ext = c.TypeExtension.BUILTIN
    num_elem = 1

    print("GIVEN:", type_except, "SELECTED:", ros_type.name)

    return (ros_type, ext, num_elem)

def get_default_val(type_name):
    if "BOOL" in type_name:
        default_val = False

    elif "BYTE" in type_name:
        default_val = b'\x00'

    elif "CHAR" in type_name:
        # There are conflicting definitions of ros2 char type
        # - http://design.ros2.org/articles/legacy_interface_definition.html
        # - https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html
        # One treats char as uint8, one as string of length 1
        # The current rosidl-py follows the first, so setting the
        # default as zero.
        default_val = 0
        # default_val = "\n"

    elif "FLOAT" in type_name:
        default_val = 0.0

    elif "INT" in type_name:
        default_val = 0

    elif "STRING" in type_name:
        default_val = ""

    return default_val

def get_bounds(type_name, req_bound):
    LOWER = 0
    UPPER = 1

    if type_name.startswith("Bool"):
        if req_bound == LOWER:
            bound = False
        elif req_bound == UPPER:
            bound = True

    elif type_name.startswith("Byte"):
        if req_bound == LOWER:
            bound = b'\x00'
        elif req_bound == UPPER:
            bound = b'\xFF'

    elif type_name.startswith("Char"):
        # There are conflicting definitions of ros2 char type
        # - http://design.ros2.org/articles/legacy_interface_definition.html
        # - https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html
        # One treats char as uint8, one as string of length 1
        # The current rosidl-py follows the first, so setting the
        # default as zero.
        if req_bound == LOWER:
            bound = 0x00
        elif req_bound == UPPER:
            bound = 0xff

    elif type_name.startswith("Float32"):
        if req_bound == LOWER:
            # bound = 1.175494351e-38
            bound = -3.4028234663852886e+38
        elif req_bound == UPPER:
            bound = 3.4028234663852886e+38

    elif type_name.startswith("Float64"):
        if req_bound == LOWER:
            # bound = 2.2250738585072014e-308
            bound = -1.7976931348623157e+308
        elif req_bound == UPPER:
            bound = 1.7976931348623157e+308

    elif type_name.startswith("Int8"):
        if req_bound == LOWER:
            bound = -128
        elif req_bound == UPPER:
            bound = 127

    elif type_name.startswith("UInt8"):
        if req_bound == LOWER:
            bound = 0
        elif req_bound == UPPER:
            bound = 255

    elif type_name.startswith("Int16"):
        if req_bound == LOWER:
            bound = -32768
        elif req_bound == UPPER:
            bound = 32767

    elif type_name.startswith("UInt16"):
        if req_bound == LOWER:
            bound = 0
        elif req_bound == UPPER:
            bound = 65535

    elif type_name.startswith("Int32"):
        if req_bound == LOWER:
            bound = -2147483648
        elif req_bound == UPPER:
            bound = 2147483647

    elif type_name.startswith("UInt32"):
        if req_bound == LOWER:
            bound = 0
        elif req_bound == UPPER:
            bound = 4294967295

    elif type_name.startswith("Int64"):
        if req_bound == LOWER:
            bound = -9223372036854775808
        elif req_bound == UPPER:
            bound = 9223372036854775807

    elif type_name.startswith("UInt64"):
        if req_bound == LOWER:
            bound = 0
        elif req_bound == UPPER:
            bound = 18446744073709551615

    elif type_name.startswith("String"):
        # bounds of utf-8 ?
        if req_bound == LOWER:
            bound = ""
        elif req_bound == UPPER:
            bound = ""

    elif type_name.startswith("WString"):
        # bounds of utf-16 ?
        if req_bound == LOWER:
            bound = ""
        elif req_bound == UPPER:
            bound = ""

    return bound


def get_rand_val(type_name):

    if type_name.startswith("Bool"):
        val = True if np.random.randint(2) else False

    elif type_name.startswith("Byte"):
        val = os.urandom(1)

    elif type_name.startswith("Char"):
        # val = str(os.urandom(1))
        # Char type is uint8 in ros2..
        val = random.randint(0, 255)

    elif type_name.startswith("Float32"):
        val = rand_float32()

    elif type_name.startswith("Float64"):
        val = rand_float64()

    elif type_name.startswith("Int8"):
        val = random.randint(-128, 127)

    elif type_name.startswith("UInt8"):
        val = random.randint(0, 255)

    elif type_name.startswith("Int16"):
        val = random.randint(-32768, 32767)

    elif type_name.startswith("UInt16"):
        val = random.randint(0, 65535)

    elif type_name.startswith("Int32"):
        val = random.randint(-2147483648, 2147483647)

    elif type_name.startswith("UInt32"):
        val = random.randint(0, 4294967295)

    elif type_name.startswith("Int64"):
        val = random.randint(-9223372036854775808, 9223372036854775807)

    elif type_name.startswith("UInt64"):
        val = random.randint(0, 18446744073709551615)

    elif type_name.startswith("String"):
        str_len_max = c.STRLEN_MAX
        str_len = np.random.randint(str_len_max)

        val = rand_unicode(str_len)

    elif type_name.startswith("WString"):
        str_len_max = c.STRLEN_MAX
        str_len = np.random.randint(str_len_max)

        val = rand_unicode(str_len)

    return val


def mutate_type():
    """ Get a random type """
    ros_type = random.choice(list(c.BuiltInType))
    ext = random.choice(list(c.TypeExtension))
    num_elem = 0
    if ext == c.TypeExtension.BUILTIN:
        num_elem = 1
    elif ext == c.TypeExtension.FARRAY:
        num_elem = c.FARRAY_BOUND
    elif ext == c.TypeExtension.BARRAY:
        num_elem = random.randint(1, c.BARRAY_BOUND_MAX)
    else:
        num_elem = random.randint(1, c.UBARRAY_BOUND_MAX)

    # print("[mutator]", ros_type, ext, num_elem)
    return (ros_type, ext, num_elem)


def mutate_one(dtype, value, stage, pos, arith_val=0, interesting_idx=-1):
    print("[*] mutate", STAGE_NAMES[stage], "on", dtype, value, "at", pos)
    if arith_val != 0:
        print("  - arith:", arith_val)
    if interesting_idx >= 0:
        print("  - interesting:", INTERESTING_MAP[stage][interesting_idx])

    if stage not in APPLICABLE_STAGES[dtype.name]:
        print("[-] FATAL: inapplicable mutation")
        return

    if pos >= BIT_LEN[dtype]:
        print("[-] pos greater than data size")
        return

    if DEBUG:
        print("=" * pos + "V" + "=" * (c.STRLEN_MAX * 8 - pos - 1))
        print(str_to_bitstr(value))

    if dtype.type is np.int8 or dtype.type is np.uint8:
        if stage <= STAGE_FLIP8:
            mask = [0] * 8
            try:
                for i in range(MUT_WINDOW_SIZE[stage]):
                    mask[pos+i] = 1
            except:
                pass
            mask = bitlist_to_binary(mask)
            value = value ^ mask

        elif stage == STAGE_ARITH8:
            return value + arith_val

        elif stage == STAGE_INTEREST8:
            return value + INTERESTING_8[interesting_idx]

    elif dtype.type is np.int16 or dtype.type is np.uint16:
        if stage <= STAGE_FLIP16:
            mask = [0] * 16
            try:
                for i in range(MUT_WINDOW_SIZE[stage]):
                    mask[pos+i] = 1
            except:
                pass
            mask = bitlist_to_binary(mask)
            value = value ^ mask

        elif stage == STAGE_ARITH8 or stage == STAGE_ARITH16:
            value_bitstr = int_to_bitstr(dtype.itemsize * 8, value)

            window = MUT_WINDOW_SIZE[stage]
            bytes_to_mutate = value_bitstr[pos:pos+window]

            if len(bytes_to_mutate) < window:
                # grabbing lsbs smaller than the window is pointless (already
                # covered during the previous arithmetic stage with half the
                # window size)
                return None
            bytes_to_mutate_int = int(bytes_to_mutate, 2)

            bytes_mutated_int = bytes_to_mutate_int + arith_val

            # drop carry bit
            bytes_mutated_bitstr = int_to_bitstr(window,
                    bytes_mutated_int)[-1*window:]

            assembled_bitstr = value_bitstr[:pos] + bytes_mutated_bitstr \
                               + value_bitstr[pos+window:]
            mut_val = int(assembled_bitstr, 2)

            return mut_val

        elif stage == STAGE_INTEREST8 or stage == STAGE_INTEREST16:
            value_bitstr = int_to_bitstr(dtype.itemsize * 8, value)

            window = MUT_WINDOW_SIZE[stage]
            bytes_to_mutate = value_bitstr[pos:pos+window]

            if len(bytes_to_mutate) < window:
                return None
            bytes_to_mutate_int = int(bytes_to_mutate, 2)
            bytes_mutated_int = bytes_to_mutate_int + INTERESTING_MAP[stage][interesting_idx]

            bytes_mutated_bitstr = int_to_bitstr(window,
                    bytes_mutated_int)[-1*window:]

            assembled_bitstr = value_bitstr[:pos] + bytes_mutated_bitstr \
                               + value_bitstr[pos+window:]
            mut_val = int(assembled_bitstr, 2)

            return mut_val

    elif dtype.type is np.int32 or dtype.type is np.uint32:
        if stage <= STAGE_FLIP32:
            mask = [0] * 32
            try:
                for i in range(MUT_WINDOW_SIZE[stage]):
                    mask[pos+i] = 1
            except:
                pass
            mask = bitlist_to_binary(mask)
            value = value ^ mask

        elif stage >= STAGE_ARITH8 and stage <= STAGE_ARITH32:
            value_bitstr = int_to_bitstr(dtype.itemsize * 8, value)

            window = MUT_WINDOW_SIZE[stage]
            bytes_to_mutate = value_bitstr[pos:pos+window]

            if len(bytes_to_mutate) < window:
                return None
            bytes_to_mutate_int = int(bytes_to_mutate, 2)

            bytes_mutated_int = bytes_to_mutate_int + arith_val
            bytes_mutated_bitstr = int_to_bitstr(window,
                    bytes_mutated_int)[-1*window:]

            assembled_bitstr = value_bitstr[:pos] + bytes_mutated_bitstr \
                               + value_bitstr[pos+window:]
            mut_val = int(assembled_bitstr, 2)

            return mut_val

        elif stage >= STAGE_INTEREST8 and stage <= STAGE_INTEREST32:
            value_bitstr = int_to_bitstr(dtype.itemsize * 8, value)

            window = MUT_WINDOW_SIZE[stage]
            bytes_to_mutate = value_bitstr[pos:pos+window]

            if len(bytes_to_mutate) < window:
                return None
            bytes_to_mutate_int = int(bytes_to_mutate, 2)

            bytes_mutated_int = bytes_to_mutate_int + INTERESTING_MAP[stage][interesting_idx]
            bytes_mutated_bitstr = int_to_bitstr(window,
                    bytes_mutated_int)[-1*window:]

            assembled_bitstr = value_bitstr[:pos] + bytes_mutated_bitstr \
                               + value_bitstr[pos+window:]
            mut_val = int(assembled_bitstr, 2)

            return mut_val

    elif dtype.type is np.int64 or dtype.type is np.uint64:
        if stage <= STAGE_FLIP32:
            mask = [0] * 64
            try:
                for i in range(MUT_WINDOW_SIZE[stage]):
                    mask[pos+i] = 1
            except:
                pass
            mask = bitlist_to_binary(mask)
            value = value ^ mask

        elif stage >= STAGE_ARITH8 and stage <= STAGE_ARITH32:
            value_bitstr = int_to_bitstr(dtype.itemsize * 8, value)

            window = MUT_WINDOW_SIZE[stage]
            bytes_to_mutate = value_bitstr[pos:pos+window]

            if len(bytes_to_mutate) < window:
                return None
            bytes_to_mutate_int = int(bytes_to_mutate, 2)

            bytes_mutated_int = bytes_to_mutate_int + arith_val
            bytes_mutated_bitstr = int_to_bitstr(window,
                    bytes_mutated_int)[-1*window:]

            assembled_bitstr = value_bitstr[:pos] + bytes_mutated_bitstr \
                               + value_bitstr[pos+window:]
            mut_val = int(assembled_bitstr, 2)

            return mut_val

        elif stage >= STAGE_INTEREST8 and stage <= STAGE_INTEREST32:
            value_bitstr = int_to_bitstr(dtype.itemsize * 8, value)

            window = MUT_WINDOW_SIZE[stage]
            bytes_to_mutate = value_bitstr[pos:pos+window]

            if len(bytes_to_mutate) < window:
                return None
            bytes_to_mutate_int = int(bytes_to_mutate, 2)

            bytes_mutated_int = bytes_to_mutate_int + INTERESTING_MAP[stage][interesting_idx]
            bytes_mutated_bitstr = int_to_bitstr(window,
                    bytes_mutated_int)[-1*window:]

            assembled_bitstr = value_bitstr[:pos] + bytes_mutated_bitstr \
                               + value_bitstr[pos+window:]
            mut_val = int(assembled_bitstr, 2)

            return mut_val


    elif dtype.type is np.float32:
        if stage <= STAGE_FLIP32:
            mask = [0] * 32
            try:
                for i in range(MUT_WINDOW_SIZE[stage]):
                    mask[pos+i] = 1
            except:
                pass
            value_bitstr = float32_to_bitstr(value)
            value_bitlist = [int(bit) for bit in value_bitstr]
            flipped = ""
            for i in range(len(mask)):
                flipped += str(value_bitlist[i] ^ mask[i])

            val_int = int(flipped, 2)
            val_bytes = val_int.to_bytes(4, "big")
            val = struct.unpack(">f", val_bytes)[0]

            return val

        elif stage >= STAGE_ARITH8 and stage <= STAGE_ARITH32:
            value_bitstr = float32_to_bitstr(value)

            window = MUT_WINDOW_SIZE[stage]
            bytes_to_mutate = value_bitstr[pos:pos+window]

            if len(bytes_to_mutate) < window:
                return None
            bytes_to_mutate_int = int(bytes_to_mutate, 2)

            bytes_mutated_int = bytes_to_mutate_int + arith_val
            bytes_mutated_bitstr = int_to_bitstr(window,
                    bytes_mutated_int)[-1*window:]

            assembled_bitstr = value_bitstr[:pos] + bytes_mutated_bitstr \
                               + value_bitstr[pos+window:]
            val_int = int(assembled_bitstr, 2)
            val_bytes = val_int.to_bytes(4, "big")
            val = struct.unpack(">f", val_bytes)[0]

            return val

        elif stage >= STAGE_INTEREST8 and stage <= STAGE_INTEREST32:
            value_bitstr = float32_to_bitstr(value)

            window = MUT_WINDOW_SIZE[stage]
            bytes_to_mutate = value_bitstr[pos:pos+window]

            if len(bytes_to_mutate) < window:
                return None
            bytes_to_mutate_int = int(bytes_to_mutate, 2)

            bytes_mutated_int = bytes_to_mutate_int + INTERESTING_MAP[stage][interesting_idx]
            bytes_mutated_bitstr = int_to_bitstr(window,
                    bytes_mutated_int)[-1*window:]

            assembled_bitstr = value_bitstr[:pos] + bytes_mutated_bitstr \
                               + value_bitstr[pos+window:]
            val_int = int(assembled_bitstr, 2)
            val_bytes = val_int.to_bytes(4, "big")
            val = struct.unpack(">f", val_bytes)[0]

            return val

    elif dtype.type is np.float64:
        if stage <= STAGE_FLIP32:
            mask = [0] * 64
            try:
                for i in range(MUT_WINDOW_SIZE[stage]):
                    mask[pos+i] = 1
            except:
                pass
            value_bitstr = float64_to_bitstr(value)
            value_bitlist = [int(bit) for bit in value_bitstr]
            flipped = ""
            for i in range(len(mask)):
                flipped += str(value_bitlist[i] ^ mask[i])

            val_int = int(flipped, 2)
            val_bytes = val_int.to_bytes(8, "big")
            val = struct.unpack(">d", val_bytes)[0]

            return val

        elif stage >= STAGE_ARITH8 and stage <= STAGE_ARITH32:
            value_bitstr = float64_to_bitstr(value)

            window = MUT_WINDOW_SIZE[stage]
            bytes_to_mutate = value_bitstr[pos:pos+window]

            if len(bytes_to_mutate) < window:
                return None
            bytes_to_mutate_int = int(bytes_to_mutate, 2)

            bytes_mutated_int = bytes_to_mutate_int + arith_val
            bytes_mutated_bitstr = int_to_bitstr(window,
                    bytes_mutated_int)[-1*window:]

            assembled_bitstr = value_bitstr[:pos] + bytes_mutated_bitstr \
                               + value_bitstr[pos+window:]
            val_int = int(assembled_bitstr, 2)
            val_bytes = val_int.to_bytes(8, "big")
            val = struct.unpack(">d", val_bytes)[0]

            return val

        elif stage >= STAGE_INTEREST8 and stage <= STAGE_INTEREST32:
            value_bitstr = float64_to_bitstr(value)

            window = MUT_WINDOW_SIZE[stage]
            bytes_to_mutate = value_bitstr[pos:pos+window]

            if len(bytes_to_mutate) < window:
                return None
            bytes_to_mutate_int = int(bytes_to_mutate, 2)

            bytes_mutated_int = bytes_to_mutate_int + INTERESTING_MAP[stage][interesting_idx]
            bytes_mutated_bitstr = int_to_bitstr(window,
                    bytes_mutated_int)[-1*window:]

            assembled_bitstr = value_bitstr[:pos] + bytes_mutated_bitstr \
                               + value_bitstr[pos+window:]
            val_int = int(assembled_bitstr, 2)
            val_bytes = val_int.to_bytes(8, "big")
            val = struct.unpack(">d", val_bytes)[0]

            return val

    elif dtype.type is np.bool_:
        if stage <= STAGE_FLIP1:
            return bool(value ^ 0b1)

    elif dtype.type is np.byte:
        if stage <= STAGE_FLIP8:
            mask = [0] * 8
            try:
                for i in range(MUT_WINDOW_SIZE[stage]):
                    mask[pos+i] = 1
            except:
                pass
            mask = bitlist_to_binary(mask)
            value = value ^ mask

        elif stage == STAGE_ARITH8:
            return value + arith_val

        elif stage == STAGE_INTEREST8:
            return value + INTERESTING_8[interesting_idx]

    elif dtype.type is np.char:
        pass

    elif dtype.type is np.str_:
        return rand_unicode(len(value))
        # if stage <= STAGE_FLIP32:
            # mask = [0] * 8 * c.STRLEN_MAX
            # try:
                # for i in range(MUT_WINDOW_SIZE[stage]):
                    # mask[pos+i] = 1
            # except:
                # pass

            # mask = bitlist_to_binary(mask)

            # value_bitstr = str_to_bitstr(value)
            # value_bitlist = [int(bit) for bit in value_bitstr]

            # value = bitlist_to_binary(value_bitlist)

            # mutated_value = value ^ mask
            # val_bytes = mutated_value.to_bytes(c.STRLEN_MAX, "big")

            # if DEBUG:
                # print(str_to_bitstr(val_bytes.decode("latin1")))
            # return val_bytes.decode("latin1")

        # elif stage >= STAGE_ARITH8 and stage <= STAGE_ARITH32:
            # value_bitstr = str_to_bitstr(value)

            # window = MUT_WINDOW_SIZE[stage]
            # bytes_to_mutate = value_bitstr[pos:pos+window]

            # if len(bytes_to_mutate) < window:
                # return None
            # bytes_to_mutate_int = int(bytes_to_mutate, 2)

            # bytes_mutated_int = bytes_to_mutate_int + arith_val
            # bytes_mutated_bitstr = int_to_bitstr(window,
                    # bytes_mutated_int)[-1*window:]

            # assembled_bitstr = value_bitstr[:pos] + bytes_mutated_bitstr \
                               # + value_bitstr[pos+window:]
            # val_int = int(assembled_bitstr, 2)
            # val_bytes = val_int.to_bytes(c.STRLEN_MAX, "big")

            # if DEBUG:
                # print(str_to_bitstr(val_bytes.decode("latin1")))
            # return val_bytes.decode("latin1")

        # elif stage >= STAGE_INTEREST8 and stage <= STAGE_INTEREST32:
            # value_bitstr = str_to_bitstr(value)

            # window = MUT_WINDOW_SIZE[stage]
            # bytes_to_mutate = value_bitstr[pos:pos+window]

            # if len(bytes_to_mutate) < window:
                # return None
            # bytes_to_mutate_int = int(bytes_to_mutate, 2)

            # bytes_mutated_int = bytes_to_mutate_int + INTERESTING_MAP[stage][interesting_idx]
            # bytes_mutated_bitstr = int_to_bitstr(window,
                    # bytes_mutated_int)[-1*window:]

            # assembled_bitstr = value_bitstr[:pos] + bytes_mutated_bitstr \
                               # + value_bitstr[pos+window:]
            # val_int = int(assembled_bitstr, 2)
            # val_bytes = val_int.to_bytes(c.STRLEN_MAX, "big")

            # if DEBUG:
                # print(str_to_bitstr(val_bytes.decode("latin1")))
            # return val_bytes.decode("latin1")

    return value

