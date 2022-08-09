import os
import sys
import signal
import subprocess as sp
import time
import json

import rclpy
from rosidl_runtime_py import message_to_ordereddict, set_message_fields


def run_px4_stack_proc():
    px4_root = "/home/seulbae/workspace/ros-security/targets/PX4-Autopilot"

    model = "iris"
    world = "fuzzing"
    bin_server = "gzserver"
    opt_server = os.path.join(
        px4_root, f"Tools/sitl_gazebo/worlds/{world}.world"
    )
    cmd1 = f"{bin_server} {opt_server}"

    bin_model = "gz model"
    opt_model = "--spawn-file={} --model-name={} -x {} -y {} -z {}".format(
        os.path.join(
            px4_root, f"Tools/sitl_gazebo/models/{model}/{model}.sdf"
        ),
        f"{model}",
        "1.01",
        "0.98",
        "0.83",
    )
    cmd2 = f"{bin_model} {opt_model}"

    env_client = os.environ.copy()
    env_client["GAZEBO_PLUGIN_PATH"] = os.path.join(
        px4_root, "build/px4_sitl_rtps/build_gazebo"
    )
    env_client["GAZEBO_MODEL_PATH"] = os.path.join(
        px4_root, "Tools/sitl_gazebo/models"
    )
    env_client["LD_LIBRARY_PATH"] = os.path.join(
        px4_root, "build/px4_sitl_rtps/build_gazebo"
    )
    bin_client = "gzclient"
    opt_client = "--gui-client-plugin libgazebo_user_camera_plugin.so"
    cmd3 = f"{bin_client} {opt_client}"

    env_px4 = os.environ.copy()
    env_px4["PX4_SIM_MODEL"] = "iris"
    bin_px4 = os.path.join(px4_root, "build/px4_sitl_rtps/bin/px4")
    etc_dir = os.path.join(px4_root, "build/px4_sitl_rtps", "etc")
    rcs = os.path.join(px4_root, "build/px4_sitl_rtps/etc/init.d-posix/rcS")
    data_dir = os.path.join(px4_root, "test_data")
    opt_px4 = f"{etc_dir} -s {rcs} -t {data_dir}"

    cwd = os.getcwd()
    wd = os.path.join(px4_root, "build/px4_sitl_rtps/tmp/rootfs")
    os.chdir(wd)
    cmd4 = f"{bin_px4} {opt_px4}"

    print(cmd1)
    pgrp1 = sp.Popen(cmd1, shell=True, preexec_fn=os.setpgrp)
    time.sleep(5)

    print(cmd2)
    pgrp2 = sp.Popen(cmd2, shell=True, preexec_fn=os.setpgrp)
    time.sleep(5)

    print(cmd3)
    pgrp3 = sp.Popen(cmd3, shell=True, preexec_fn=os.setpgrp, env=env_client)
    time.sleep(5)

    print(cmd4)
    pgrp4 = sp.Popen(cmd4, shell=True, preexec_fn=os.setpgrp, env=env_px4)
    time.sleep(5)

    os.killpg(pgrp1.pid, signal.SIGKILL)
    os.killpg(pgrp2.pid, signal.SIGKILL)
    os.killpg(pgrp3.pid, signal.SIGKILL)
    os.killpg(pgrp4.pid, signal.SIGKILL)


def run_px4_stack_sh(proj_dir):
    px4_dir = os.path.join(proj_dir, "targets", "PX4-Autopilot")
    # use below if measuring coverage:
    # px4_dir = "/home/seulbae/workspace/px4-cov"
    devnull = "2>&1 > /dev/null"

    simulator = "gazebo"
    model = "iris"

    cmd_list = [
        "PX4_SITL_WORLD=church",
        os.path.join(px4_dir, "Tools", "sitl_run.sh"),
        os.path.join(px4_dir, "build", "px4_sitl_rtps", "bin", "px4"),
        "none",
        simulator,
        model,
        "none",
        px4_dir,
        os.path.join(px4_dir, "build", "px4_sitl_rtps"),
        devnull,
    ]
    cmd = " ".join(cmd_list)
    # print("command:", cmd)

    proc = sp.Popen(cmd, shell=True, stdout=sp.DEVNULL, stderr=sp.DEVNULL)
    return proc


def run_tb3_sitl(proj_dir):
    tb3_dir = os.path.join(proj_dir, "targets", "turtlebot3_ws")
    devnull = "2>&1 > /dev/null"

    ros_pkg = "turtlebot3_gazebo"
    sim_map = "turtlebot3_world.launch.py"

    cmd_list = [
        f"DISPLAY={os.getenv('DISPLAY')}",
        "TURTLEBOT3_MODEL=burger",
        "ros2",
        "launch",
        ros_pkg,
        sim_map,
    ]
    cmd = " ".join(cmd_list)
    # print("command:", cmd)

    pgrp = sp.Popen(
        cmd,
        shell=True,
        preexec_fn=os.setpgrp,
        stdout=sp.DEVNULL,
        stderr=sp.DEVNULL,
    )
    return pgrp


def run_tb3_hitl(tb3_uri):
    cmd = f"ssh -i keys/tb3 {tb3_uri} ./run.sh"

    proc = sp.Popen(cmd, shell=True, stdout=sp.DEVNULL, stderr=sp.DEVNULL)
    return proc


def run_rcl_api_harness(features, targets, job):
    feature_str = " ".join(features)
    target_str = " ".join(targets)

    cmd = f"python3 rcl_harness.py -f {feature_str} -t {target_str} -j {job}"

    pgrp = sp.Popen(
        cmd,
        shell=True,
        preexec_fn=os.setpgrp,
        stdout=sys.stdout,
        stderr=sys.stderr,
    )

    return pgrp


def run_cli_harness():
    cmd = "python3 cli_harness.py"

    pgrp = sp.Popen(
        cmd,
        shell=True,
        preexec_fn=os.setpgrp,
        stdout=sys.stdout,
        stderr=sys.stderr,
    )

    return pgrp


def run_rosidl_harness(lang, shmid, ros_type="empty"):
    # lang, shm, ros_type
    # need shm for both lang targets as they need to fetch data from shm

    # if lang == "py":
        # cmd = ""
    # elif lang == "cpp":
        # cmd = ""

    cmd = "ros2 run idltest_target idltest_target"

    pgrp = sp.Popen(
        cmd,
        shell=True,
        preexec_fn=os.setpgrp,
        stdout=sys.stdout,
        stderr=sys.stderr,
    )

    return pgrp


def run_moveit_harness():
    cmd = f"DISPLAY={os.getenv('DISPLAY')} ros2 launch moveit2_tutorials move_group.launch.py 2>&1 > /dev/null"

    pgrp = sp.Popen(
        cmd,
        shell=True,
        preexec_fn=os.setpgrp,
        stdout=sys.stdout,
        stderr=sys.stderr,
    )

    return pgrp


def get_init_moveit_msg():
    from moveit_msgs.msg import MotionPlanRequest

    # initial (ready) position
    with open("moveit_panda_msg.json", "r") as f:
        msg_json = json.load(f)

    msg = MotionPlanRequest()
    set_message_fields(msg, msg_json)

    # re-assign timestamp
    # ts = time.time_ns()
    # sec = int(ts / pow(10, 9))
    # nsec = ts - sec * pow(10, 9)
    # msg.workspace_parameters.header.stamp.sec = sec
    # msg.workspace_parameters.header.stamp.nanosec = nsec

    return msg


def get_init_joint_constraints():
    msg = get_init_moveit_msg()
    joint_constraints = msg["goal_constraints"][0]["joint_constraints"]
    return joint_constraints


def get_init_moveit_pose():
    from geometry_msgs.msg import Pose
    msg = Pose()
    # msg.position.x = 0.28
    # msg.position.y = -0.2
    # msg.position.z = 0.5
    # msg.orientation.w = 1.0
    msg.position.x = 0.5
    msg.position.y = 0.5
    msg.position.z = 0.5
    msg.orientation.w = 1.0

    return msg

def moveit_send_command(msg):
    print("[moveit harness] sending goal command")
    x = str(msg.position.x)
    y = str(msg.position.y)
    z = str(msg.position.z)
    w = str(msg.orientation.w)

    cmd = "ros2 launch moveit2_tutorials move_group_interface_tutorial.launch.py"

    sp.call(
        [
            "ros2",
            "launch",
            "moveit2_tutorials",
            "move_group_interface_tutorial.launch.py",
            f"x:={x}",
            f"y:={y}",
            f"z:={z}",
            f"w:={w}",
        ],
        stdout=sp.DEVNULL,
        stderr=sp.DEVNULL,
    )
    print("                 + sent")


if __name__ == "__main__":
    # run_px4_stack_sh()
    # pgrp = run_tb3_sitl(os.path.join(os.getcwd(), "../"))
    pgrp = run_moveit2_harness()
    print("pgroup pid:", pgrp.pid)
    time.sleep(20)
    try:
        os.killpg(pgrp.pid, signal.SIGKILL)
    except:
        print("err")
