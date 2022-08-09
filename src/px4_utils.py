import os
import sys
import time
import json
import pickle

import rclpy
from px4_msgs.msg import (
    Timesync,
    VehicleCommand,
    TrajectorySetpoint,
    OffboardControlMode,
    ManualControlSetpoint,
)
from rosidl_runtime_py import message_to_ordereddict, set_message_fields
import rosidl_parser.definition

# force MAVLink 2.0
os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil


# TODO: add all parameters and their metadata
# params related to the bugs pgfuzz found first
param_dict = pickle.load(open("px4_prep/params.pkl", "rb"))
# {name: type, min, max, default, unit}

# param_dict = {
    # "MIS_TAKEOFF_ALT": [0, 2.5, 80],
    # "COM_POS_FS_EPH": [None, 5, None],
    # "EKF2_ABL_LIM": [0.0, 0.0, None],
    # "MC_PITCHRATE_FF": [0.0, 0.0, None],
# }


class Parameter:
    """Class defining a PX4 parameter message"""
    __slots__ = [
        "param_name", "param_type", "min", "max", "_value",
    ]

    _fields_and_field_types = {
        'value': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, param_name=None, value=None):
        self.param_name = param_name
        self.value = value

    def __repr__(self):
        return f"PX4 Param {self.param_name} ({self.param_type}): {self.value} [{self.min}:{self.max}]"

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        self._value = value


class Px4BridgeNode:
    def __init__(self, name="px4_bridge", use_mavlink=True, debug=False):
        self.use_mavlink = use_mavlink
        self.debug = debug

        if not self.use_mavlink:
            self.node_handle = rclpy.create_node(name)

            self.sub_timesync = self.node_handle.create_subscription(
                Timesync, "Timesync_PubSubTopic", self.timesync_callback, 10
            )

            self.pub_offboard_control_mode = self.node_handle.create_publisher(
                OffboardControlMode, "/OffboardControlMode_PubSubTopic", 10
            )
            self.pub_vehiclecommand = self.node_handle.create_publisher(
                VehicleCommand, "/VehicleCommand_PubSubTopic", 10
            )
            self.pub_trajectory = self.node_handle.create_publisher(
                TrajectorySetpoint, "/TrajectorySetpoint_PubSubTopic", 10
            )

    def init_mavlink(self):
        self.use_mavlink = True
        # using mavlink for generic communication
        self.master = mavutil.mavlink_connection("udpin:127.0.0.1:14550")
        self.master.wait_heartbeat()

    def timesync_callback(self, msg):
        self.ts = msg.timestamp

    def get_timestamp(self):
        return self.ts

    def publish_offboard_mode(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.timestamp = self.ts
        msg.param1 = 1.0
        msg.param2 = 6.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        # print(msg)
        self.pub_vehiclecommand.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.ts
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        # print(msg)
        self.pub_offboard_control_mode.publish(msg)

    def publish_arm_command(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.timestamp = self.ts
        msg.param1 = 1.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        # print(msg)
        self.pub_vehiclecommand.publish(msg)

    def publish_trajectory(self, msg):
        # msg = TrajectorySetpoint()
        msg.timestamp = self.ts

        # msg.x = float(random.randint(-1000, 1000)) / random.randint(1, 5)
        # msg.y = float(random.randint(-1000, 1000)) / random.randint(1, 5)
        # msg.z = float(random.randint(-100, -1)) / random.randint(1, 5)
        # msg.x = 0.0
        # msg.y = 0.0
        # msg.z = -2.0

        # if self.bit:
        # msg.yaw = -3.14
        # else:
        # msg.yaw = 0.0
        # self.bit ^= 1
        # msg.yaw = 0.0
        # msg.yawspeed = msg.yawspeed

        # msg.vx = 0.0
        # msg.vy = 0.0
        # msg.vz = 0.0

        # print(msg)
        self.pub_trajectory.publish(msg)

    def mav_get_param_default(self, param_name):
        return param_dict[param_name]["default"]

    def mav_set_param_msg(self, msg):
        # wrapper for pub_function
        if msg.param_type == "INT32":
            mav_param_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32
        elif msg.param_type == "FLOAT":
            mav_param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32

        self.mav_set_param(msg.param_name, msg.value, mav_param_type)

    def mav_set_param(self, param_name, param_value, param_type):
        print(f"[*] (MAVLink) set param {param_name} := {param_value}")
        self.current_param_name = param_name
        self.current_param_type = param_type

        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            param_name.encode(), # requires 'b'
            param_value,
            param_type,
        )

        # ack_msg = self.master.recv_match(type='PARAM_VALUE', blocking=True)
        # ack_msg = ack_msg.to_dict()
        # print("param set ack:")
        # print(ack_msg)

        if self.debug:
            # Verify parameter change
            self.master.mav.param_request_list_send(
                self.master.target_system,
                self.master.target_component
            )

            while True:
                time.sleep(0.01)
                try:
                    message = self.master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()

                    if message["param_id"] == param_name:
                        print('name: %s\tvalue: %d' % (message['param_id'],
                                                       message['param_value']))
                        break

                except Exception as error:
                    print(error)
                    sys.exit(0)

    def mav_revert_param(self):
        param_name = self.current_param_name
        param_type = self.current_param_type
        value = self.mav_get_param_default(param_name)

        self.mav_set_param(param_name, value, param_type)

    def prepare_flight(self, mode_str="MANUAL"):

        if self.use_mavlink:
            # Set COM_RC_LOSS_T = 30 to avoid engaging failsafe
            print("[*] (MAVLink) requesting COM_RC_LOSS_T override")
            self.mav_set_param(
                "COM_RC_LOSS_T",
                30,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

            # Send dummy manual control command to keep the RC connected
            msg = get_init_manual_control_msg()
            self.send_command(msg)

            # mode change won't work without this
            time.sleep(1)

            # Set mode
            self.mav_set_flight_mode(mode_str=mode_str)

            # arm
            self.send_command_arm()

        else:
            # The vehicle must be already be receiving a stream of target
            # setpoints (>2Hz) before this mode can be engaged.
            dummy = TrajectorySetpoint()
            dummy.z = -0.5
            for i in range(50):
                rclpy.spin_once(self.node_handle)
                self.publish_offboard_control_mode()
                self.publish_trajectory(dummy)
                time.sleep(0.1)

            rclpy.spin_once(self.node_handle)
            self.publish_offboard_mode()
            time.sleep(0.1)

            # The vehicle must be armed before this mode can be engaged.
            rclpy.spin_once(self.node_handle)
            self.publish_arm_command()
            time.sleep(0.1)

    def send_command_arm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )

        self.master.motors_armed_wait()
        time.sleep(1)

    def send_command_takeoff(self, alt=10.0):
        print(f"[+] (MAVLink) send takeoff command")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            alt
        )
        time.sleep(1)

    def send_command(self, msg):
        if self.use_mavlink:
            # print("[mavlink] sending:", msg)
            self.master.mav.manual_control_send(
                self.master.target_system,
                int(msg.x * 1000),
                int(msg.y * 1000),
                int(msg.z * 1000),
                int(msg.r * 1000),
                0
            )

        else:
            rclpy.spin_once(self.node_handle)
            self.publish_offboard_control_mode()
            self.publish_trajectory(msg)
            print("[offboard controller] sending:", msg)

    def put_in_air(self):
        for i in range(100):
            self.master.mav.manual_control_send(
                self.master.target_system,
                0,
                0,
                800,
                0,
                0
            )
            time.sleep(0.1)
        time.sleep(1)

    def mav_set_flight_mode(self, mode_str="MANUAL"):
        # Set manual mode
        (mode, custom_mode, custom_sub_mode) = self.master.mode_mapping()[mode_str]

        self.master.set_mode(mode, custom_mode, custom_sub_mode)

        # cnt = 0
        # while True:
            # cnt += 1
            # if cnt == 100:
                # print(f"[-] (MAVLink) timeout while waiting for set_mode ack")
                # sys.exit(-1)

            # ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
            # ack_msg = ack_msg.to_dict()

            # if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                # continue

            # if ack_msg["result"] == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                # print(f"[+] (MAVLink) mode set: {mode_str}")
                # break
            # else:
                # print(f"[-] (MAVLink) failed to set mode {mode_str}")
                # print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
                # sys.exit(-1)
        time.sleep(1)

    def mav_get_status_dict(self):
        try:
            status_dict = self.master.recv_match().to_dict()
        except:
            status_dict = None

        return status_dict

    def takeoff(self):
        print("[offboard controller] take off")
        msg_takeoff = TrajectorySetpoint()
        msg_takeoff.z = -5.0
        for i in range(100):
            self.send_command(msg_takeoff)

    def land(self):
        print("[offboard controller] land")
        msg_land = TrajectorySetpoint()
        msg_land.z = 0.0
        for i in range(100):
            self.send_command(msg_land)

    def shutdown(self):
        self.node_handle.destroy_node()


def get_init_trajectory_msg():
    msg = TrajectorySetpoint()
    msg.x = 0.0
    msg.y = 0.0
    msg.z = -5.0

    return msg


def get_init_manual_control_msg():
    msg = ManualControlSetpoint()

    # corresponds to joystick position
    msg.x = 0.0 # pitch [-1:1]
    msg.y = 0.0 # roll [-1:1]
    msg.z = 0.75 # throttle [0:1] - 0.5 is neutral, but gently throttle upwards
    msg.r = 0.0 # yaw [-1:1]

    return msg


def get_init_parameter_msg():
    msg = Parameter()
    msg.param_name = "MIS_TAKEOFF_ALT"
    msg.value = 2.5

    return msg


def read_trajectory_seed(seedfile):
    """seedfile contains either a msg in a dictionary (single repeated mode)
    or a list of msg dictionaries (interception mode), depending on the desired mode"""

    if not os.path.isfile(seedfile):
        return None

    with open(seedfile, "r") as fp:
        json_content = json.load(fp)

    if type(json_content) is dict:
        msg = TrajectorySetpoint()
        set_message_fields(msg, json_content)

        return msg

    elif type(json_content) is list:
        trajectory_msg_list = []
        for msg_dict in json_content:
            msg = TrajectorySetpoint()
            set_message_fields(msg, msg_dict)
            trajectory_msg_list.append(msg)

        return trajectory_msg_list

    else:
        print("[-] unknown seed type")
        return None


def read_offboard_mission(missionfile):

    if not os.path.isfile(missionfile):
        return None

    with open(missionfile, "r") as fp:
        msg_list = json.load(fp)

    trajectory_msg_list = []
    for msg_dict in msg_list:
        msg = TrajectorySetpoint()
        set_message_fields(msg, msg_dict)
        trajectory_msg_list.append(msg)

    return trajectory_msg_list


def conduct_offboard_mission(bridge, mission):

    bridge.takeoff()

    for msg in mission:
        bridge.send_command(msg)

    bridge.land()
