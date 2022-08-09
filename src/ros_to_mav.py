import time
import sys
import os

# force MAVLink 2.0
os.environ["MAVLINK20"] = "1"

# doc: https://mavlink.io/en/mavgen_python/
from pymavlink import mavutil

# Create a function to send RC values
# More information about Joystick channels
# here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
def set_rc_channel_pwm(channel_id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """

    """
    1 pitch
    2 roll
    3 throttle
    4 yaw
    5 forward
    6 lateral
    7 camera pan
    8 camera tilt
    9 lights 1 level
    10 lights 2 level
    11 video switch

    [RC Mode 2]
     ^ throttle  ^ pitch
    < > yaw     < > roll
     v           v

    """

    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.


if __name__ == "__main__":

    master = mavutil.mavlink_connection("udpin:127.0.0.1:14550")

    # make sure the connection is valid
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
            (master.target_system, master.target_component))

    print(master.__dict__)
    print("-----")

    # set / connect (virtual) RC before arming to prevent px4 from
    # engaging the failsafe mode right away

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )

    print("waiting for the vehicle to arm")
    master.motors_armed_wait()
    print("armed!")

    # ack = False
    # while not ack:
        # # Wait for ACK command
        # ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        # ack_msg = ack_msg.to_dict()

        # print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        # break

    time.sleep(1)

    # # Request all parameters
    # master.mav.param_request_list_send(
        # master.target_system, master.target_component
    # )
    # while True:
        # # time.sleep(0.01)
        # try:
            # message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
            # print('name: {}\tvalue: {}'.format(message['param_id'],
                                           # message['param_value']))
        # except Exception as error:
            # print(error)
            # sys.exit(0)

    # print("end")

    # # px4 verification
    # mav_type = master.sysid_state[master.sysid].mav_type
    # mav_autopilot = master.sysid_state[master.sysid].mav_autopilot
    # print(mav_autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4)

    """
    {'MANUAL': (81, 1, 0), 'STABILIZED': (81, 7, 0), 'ACRO': (65, 5, 0), 'RATTITUDE': (65, 8, 0), 'ALTCTL': (81, 2, 0), 'POSCTL': (81, 3, 0), 'LOITER': (29, 4, 3), 'MISSION': (29, 4, 4), 'RTL': (29, 4, 5), 'LAND': (29, 4, 6), 'RTGS': (29, 4, 7), 'FOLLOWME': (29, 4, 8), 'OFFBOARD': (29, 6, 0), 'TAKEOFF': (29, 4, 2)}
    """

    mode_str = "MANUAL"
    (mode, custom_mode, custom_sub_mode) = master.mode_mapping()[mode_str]

    master.set_mode(mode, custom_mode, custom_sub_mode)

    while True:
        # Wait for ACK command
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()
        print("mode ack:", ack_msg)

        # Check if command in the same in `set_mode`
        if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            continue

        # Print the ACK result !
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        break

    print(f"mode set to {mode_str}")

    time.sleep(5)

    for i in range(1000):
        master.mav.manual_control_send(
            master.target_system,
            0, # x
            0,# y
            1000, # z
            0, # r
            0)
        time.sleep(0.01)

    for i in range(1000):
        master.mav.manual_control_send(
            master.target_system,
            0, # x
            0,# y
            50, # z
            0, # r
            0)
        time.sleep(0.01)

    # time.sleep(10)

    # set_rc_channel_pwm(3, 1900)

    # # https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF
    # master.mav.command_long_send(
        # master.target_system,  # target_system
        # master.target_component,  # target_component
        # mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # command (22)
        # 0,  # confirmation
        # 0,  # param1 - pitch (deg)
        # 0,  # param2 - empty
        # 0,  # param3 - empty
        # 0,  # param4 - yaw angle (deg)
        # 0,  # param5 - lat
        # 0,  # param6 - lon
        # 100)  # param7 - altitude (m)

    # ack = False
    # while not ack:
        # # Wait for ACK command
        # ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        # ack_msg = ack_msg.to_dict()
        # print("takeoff ack:", ack_msg)

        # if ack_msg['command'] != mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            # continue

        # print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        # break

    # print("takeoff command acked")
    # time.sleep(25)

    while True:
        print("blocking")
        time.sleep(2)
