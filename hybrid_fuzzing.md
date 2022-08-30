# HowTo: Hybrid Fuzzing
Requirements for running RoboFuzz with an actual robot differs per robot. In
this document, we provide instructions and a demonstration to shed light on
the setup procedure for hybrid fuzzing.

## 1. Preparation (HW)
First, prepare the physical robot. In this document,
We will use [TurtleBot3 Burger (TB3)](https://www.robotis.us/turtlebot-3-burger-us/)
as an example.

Make sure that all parts are assembled, and that you can connect to the
robot over the network (e.g., `ssh`). For ROS 2 messages to be delivered
from the fuzzer (i.e., PC) to the robot and vice versa, both the PC and the
robot should be on the same sub-network, e.g., connected to the same wifi
router.

We created a ssh key to automate the remote access and created two shell
scripts, `run.sh` and `kill.sh`. `run.sh` sets up ROS 2 environment and
launches TB3 via `ros2 launch`. `kill.sh` kills all ROS 2 processes.

```sh
host_machine:~$ ssh -i path_to_robofuzz/src/keys/tb3 ubuntu@ip_addr_of_TB3

ubuntu@ubuntu:~$ cat run.sh
#!/bin/bash
source /opt/ros/foxy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py

ubuntu@ubuntu:~$ cat kill.sh
#!/bin/bash
kill -9 $(ps aux | grep ros | awk '{print $2}')
```

## 2. Preparation (SW)
On the software side, prepare the simulation following the [manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/).

ROS2 provides "topic remapping and namespacing", which allows you to group
topics via namespace when you have multiple instances of robots. Put
state-related topics under a namespace to differentiate them from the topics
used by the physical robot. In the case of TB3, from `https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git`,
change the following to put simulation topics under the namespace `/sim`:
```sh
diff --git a/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf b/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf
index 562cac0..ac81e1a 100644
--- a/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf
+++ b/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf
@@ -88,7 +88,7 @@
         </imu>
         <plugin name="turtlebot3_imu" filename="libgazebo_ros_imu_sensor.so">
           <ros>
-            <!-- <namespace>/tb3</namespace> -->
+            <namespace>/sim</namespace>
             <remapping>~/out:=imu</remapping>
           </ros>
         </plugin>
@@ -156,7 +156,7 @@
         </ray>
         <plugin name="turtlebot3_laserscan" filename="libgazebo_ros_ray_sensor.so">
           <ros>
-            <!-- <namespace>/tb3</namespace> -->
+            <namespace>/sim</namespace>
             <remapping>~/out:=scan</remapping>
           </ros>
           <output_type>sensor_msgs/LaserScan</output_type>
@@ -365,7 +365,8 @@
     <plugin name="turtlebot3_diff_drive" filename="libgazebo_ros_diff_drive.so">

       <ros>
-        <!-- <namespace>/tb3</namespace> -->
+        <namespace>/sim</namespace>
+        <remapping>/sim/cmd_vel:=/cmd_vel</remapping>
       </ros>

       <update_rate>30</update_rate>
@@ -397,7 +398,7 @@

     <plugin name="turtlebot3_joint_state" filename="libgazebo_ros_joint_state_publisher.so">
       <ros>
-        <!-- <namespace>/tb3</namespace> -->
+        <namespace>/sim</namespace>
         <remapping>~/out:=joint_states</remapping>
       </ros>
       <update_rate>30</update_rate>
```

## 3. Preparation (Fuzzer)

Your watchlist should list the simulation topics as well as the actual
robot's topics:
```sh
$ cat watchlist/turtlebot3_hybrid.json
{
    "/cmd_vel": "geometry_msgs/msg/Twist",
    "/imu": "sensor_msgs/msg/Imu",
    "/odom": "nav_msgs/msg/Odometry",
    "/scan": "sensor_msgs/msg/LaserScan",
    "/sim/imu": "sensor_msgs/msg/Imu",
    "/sim/odom": "nav_msgs/msg/Odometry",
    "/sim/scan": "sensor_msgs/msg/LaserScan"
}
```

The IMU, odometry, and LiDAR scan data of the physical robot will be
published to `/imu`, `/odom`, and `/scan` (by default) and the simulated one
will do the same to `/sim/imu`, `/sim/odom`, and `/sim/scan`.
All states of the physical robot and the simulated robot will be recorded.

Write an oracle that reads from all aforementioned topics (see our oracle
examples) and runs checks for the correctness properties you want to test.

Create a harness function for starting both the simulator and the physical
robot referring to `run_tb3_sitl()` and `run_tb3_hitl()` in `src/harness.py`.
Register your harness function in `run_target` and `kill_target` of
`src/fuzzer.py`, and you are all set!

## Video demo
You can find a video demonstration of the TB3 bug found in the hybrid mode [here](https://youtu.be/MB5iCiYLBCI).

## Troubleshooting
Please feel free to open a Github issue if you need additional assistance.

