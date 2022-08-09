from enum import Enum, auto

ros_time_types = ['time', 'duration']
ros_primitive_types = ['bool', 'byte', 'char', 'int8', 'uint8', 'int16',
        'uint16', 'int32', 'uint32', 'int64', 'uint64',
        'float32', 'float64',
        'string']
ros_header_types = ['Header', 'std_msgs/Header', 'roslib/Header']

# fuzzing mode
M_STARTOVER = 0
M_STATEFUL  = 1

STRLEN_MAX = 16

# colors
RED = "\x1b[31m"
GREEN = "\x1b[92m"
END = "\x1b[0m"

# lock files
PUBLOCK = ".publish.lock"

WATCHFLAG = ".watch"

# IDL-related
class BuiltInType(Enum):
    BOOL = auto()
    # BYTE = auto() # https://github.com/ros2/rosidl_runtime_py/issues/14
    CHAR = auto()
    FLOAT32 = auto()
    FLOAT64 = auto()
    INT8 = auto()
    UINT8 = auto()
    INT16 = auto()
    UINT16 = auto()
    INT32 = auto()
    UINT32 = auto()
    INT64 = auto()
    UINT64 = auto()
    STRING = auto()
    WSTRING = auto()


class TypeExtension(Enum):
    BUILTIN = auto() # single built-in type
    FARRAY = auto() # array of fixed size
    BARRAY = auto() # bounded array of built-in types
    UBARRAY = auto() # unbounded array of built-in types

# Cannot find the maximum allowed num of elements of an unbounded array
# probably depends on the DDS implementation.
# ref: https://answers.ros.org/question/299322/maximum-samples-and-unbounded-type-length-in-ros2-dds-entity/
FARRAY_BOUND = 64
BARRAY_BOUND_MAX = 64
UBARRAY_BOUND_MAX = 16384 # bigger numbers result in "arg list too long" error

###
