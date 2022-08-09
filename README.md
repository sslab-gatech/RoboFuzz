# RoboFuzz

RoboFuzz is a fuzzing framework for testing Robot Operating System 2 (ROS 2),
and robotic systems that are built using ROS 2. Any developer-defined
properties relating to the correctness of the robotic system under test,
e.g., conformance to specification, can be tested using RoboFuzz.

The paper "RoboFuzz: Fuzzing Robotic Systems over Robot Operating System (ROS)
for Finding Correctness Bugs" will be published at ESEC/FSE 2022 in November.

We tested six targets with RoboFuzz;

* Two from the internal layers of ROS2 foxy:
  1. Type system (ROSIDL)
  2. ROS Client Library APIs (rclpy and rclcpp)

* Four ROS-based robotic systems/libraries:
  3. Turtlesim (apt pkg: ros-foxy-turtlesim)
  4. Move It 2 + PANDA manipulator (included in moveit2_tutorials)
  5. TurtleBot3 Burger (ver. foxy)
  6. PX4 quadcopter (firmware v1.12 + fmu-v5)

We built test oracles by studying and encoding the correctness semantics
of each target. These oracles are capable of detecting three types of semantic
correctness bugs:
1. Violation of physical laws
2. Violation of specification
3. Cyber-physical discrepancy


## Getting RoboFuzz

Before you start, please refer to `REQUIREMENTS.md` for hardware and software
requirements, and check `INSTALL.md` for the instructions on the installation
and basic usage.

In a nutshell, you can obtain and use RoboFuzz by getting the docker image:
```
$ docker pull ghcr.io/sslab-gatech/robofuzz:latest
$ docker tag ghcr.io/sslab-gatech/robofuzz:latest robofuzz
```
(Due to the pre-compiled target robotic systems, the size of the image is
 approximately 20 GB, so docker pull might take a while.)

The docker image includes the code of RoboFuzz, along with the pre-compiled
packages of the six targets and all the tools required for experiments:
* `/robofuzz/src/`: Source code of RoboFuzz
  * `/robofuzz/src/fuzzer.py`: Main module of RoboFuzz
  * `/robofuzz/src/oracles/*`: Correctness oracles for target systems
  * `/robofuzz/src/watchlist/*`: Topics to monitor
* `/robofuzz/targets`: Pre-compiled test targets
* `/robofuzz/ros2_foxy`: ROS2 foxy installation (instrumented ver.)
* `/opt/ros/foxy`: ROS2 foxy installation (package manager ver.)


## Running RoboFuzz and reproducing our results

RoboFuzz needs to be run separately for each target system.

### 0. Understanding the output of RoboFuzz

RoboFuzz creates an output directory to log all runtime information and
metadata. Unless the path is explicitly specified through `--logdir LOGDIR`,
you can find the logs under `/robofuzz/src/logs/timestamp` in the container.
In addition, `/robofuzz/src/logs/latest` always points (i.e., symlinks) to the
latest log directory.

* Glossary
  * Exec: One exec of fuzzing includes running the target system, publishing
           a mutated message, checking for errors, and terminating the target.
  * Round: A round of fuzzing consists of multiple executions under a specific
           mutation schedule. Check `/robofuzz/src/scheduler.py` for details.
  * Cycle: A cycle is a collection of several rounds. The number of rounds in
           a cycle is configurable. In general, RoboFuzz keeps mutating one
           message during one cycle, and moves on to the next message in the
           queue when the new cycle begins.
  * Frame: The timestamp taken at the beginning of a round.
  * Subframe: The timestamp taken when each message is published. For example,
              if an execution begins at `ts-0`, it is the frame. If, during
              the execution, RoboFuzz publishes a sequence of three messages
              at `ts-1`, `ts-2`, and, `ts-3` respectively, each timestamp is
              the subframe of the corresponding message. Using the combination
              of the frame and subframe, any message can be located.

* Each log directory has several sub-directories:
  * `metadata/`: contains node, topic, data type, cycle and round count of
                 each execution.
    * Filename format: `meta-{Frame}`
  * `errors/`: Error messages emitted by oracles when bugs are triggered.
    * Filename format: `error-{Frame}`.
  * `cov/`: Feedback elements and their values when they are considered
            interesting.
    * Filename format: `{Frame}`.
  * `queue/`: Messages generated/mutated during fuzzing. If multiple messages
              were published during one execution, e.g., with `SEQUENCE`
              schedule, they can be retrieved by `ls queue | grep {Frame}`.
    * Filename format: `msg-{Frame}-{Subframe}`.
  * `rosbags/`: Rosbag files (i.e., dump of all messages and states) of buggy
                test cases. These are standard rosbags of ROS 2, and for
                detailed usage, please refer to the [official tutorial](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html).
    * Filename format: `{Frame}/states-{exec}.bag`

During or after the fuzzing campaign, you can refer to the contents of the log
directory to make sense of the fuzzing progress and detected bugs.

### 1. Testing ROSIDL

1. Start RoboFuzz container (HOST)
    ```
    $ xhost +
    $ docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --name robofuzz robofuzz
    root@container_id:/robofuzz/src#
    ```

2. Run RoboFuzz (CONTAINER)
    ```
    root@container_id:/robofuzz/src# ./fuzzer.py --test-rosidl --no-cov --watchlist watchlist/idltest.json
    ```

The fuzzer will cycle through various ROS types and test the correctness of
the type system implementation.

### 2. Testing RCL API consistency

1. Start RoboFuzz container (HOST)
    ```
    $ xhost +
    $ docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --name robofuzz robofuzz
    root@container_id:/robofuzz/src#
    ```

2. Run RoboFuzz (CONTAINER)
    Please note that each of the following substep should be run separately
    in a clean-slate RoboFuzz container.

    a. Testing RCL publisher APIs while creating a publisher
      ```
      root@container_id:/robofuzz/src# source /robofuzz/ros2_foxy/install/setup.bash
      root@container_id:/robofuzz/src# ./fuzzer.py --test-rcl --rcl-api publisher --rcl-job create_publisher
      ```
    b. Testing RCL subscriber APIs while creating a subscriber
      ```
      root@container_id:/robofuzz/src# source /robofuzz/ros2_foxy/install/setup.bash
      root@container_id:/robofuzz/src# ./fuzzer.py --test-rcl --rcl-api subscriber --rcl-job create_subscriber
      ```

    c. Testing RCL node APIs while creating a node
      ```
      root@container_id:/robofuzz/src# source /robofuzz/ros2_foxy/install/setup.bash
      root@container_id:/robofuzz/src# ./fuzzer.py --test-rcl --rcl-api node --rcl-job create_node
      ```

    d. Testing RCL + CLI API consistency while setting a parameter
      ```
      root@container_id:/robofuzz/src# source /robofuzz/ros2_foxy/install/setup.bash
      root@container_id:/robofuzz/src# ./fuzzer.py --test-cli --no-cov
      ```

### 3. Testing Turtlesim

1. Start RoboFuzz container (HOST)
    ```
    $ xhost +
    $ docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --name robofuzz robofuzz
    root@container_id:/robofuzz/src#
    ```

2. Run RoboFuzz (CONTAINER)
    ```
    root@container_id:/robofuzz/src# ./fuzzer.py --no-cov --ros-pkg turtlesim --ros-node turtlesim_node --watchlist watchlist/turtlesim.json --method message --schedule single --interval 0.1
    ```

### 4. Testing Move It 2 + PANDA manipulator

1. Start RoboFuzz container (HOST)
    ```
    $ xhost +
    $ docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --name robofuzz robofuzz
    root@container_id:/robofuzz/src#
    ```

2. Run RoboFuzz (CONTAINER)
    ```
    root@container_id:/robofuzz/src# ./fuzzer.py --test-moveit --watchlist watchlist/moveit2.json --no-cov
    ```

Please note that we set a small margin in the joint limit checker oracle to
prevent an [existing specification violation](https://github.com/ros-planning/moveit_resources/issues/116)
that we found from being constantly reported and overshadowing other bugs.
To reproduce this bug, please open `/robofuzz/src/oracle/moveit.py` and
override `MARGIN` to `0.0` (see comments at the beginning of the file.)

### 5. Testing TurtleBot3 Burger

1. Start RoboFuzz container (HOST)
    ```
    $ xhost +
    $ docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --name robofuzz robofuzz
    root@container_id:/robofuzz/src#
    ```

2. Run RoboFuzz (CONTAINER)
    ```
    root@container_id:/robofuzz/src# ./fuzzer.py --tb3-sitl --no-cov --method message --schedule single --repeat 1 --interval 5.0 --watchlist watchlist/turtlebot3.json
    ```

Like the case of PANDA manipulator, there is an [existing specification violation error](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/178)
in the LiDAR processor node, and this bug is triggered all the time regardless
of the input messages, i.e., `scan.range inf is out of range`. To suppress the
bug, please comment out lines 242-259 in `/robofuzz/src/oracle/turtlebot.py`
and re-run the fuzzer.

### 6. Testing PX4 quadcopter

1. Start RoboFuzz container (HOST, terminal 1)
    ```
    $ xhost +
    $ docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --name robofuzz robofuzz
    root@container_id:/robofuzz/src#
    ```

2. On another terminal, attach to the running container and run
   `micrortps_agent` (HOST->CONTAINER, terminal 2)
    ```
    $ docker exec -ti robofuzz /bin/bash
    root@container_id:/robofuzz/src# source /ros_entrypoint.sh
    root@container_id:/robofuzz/src# micrortps_agent -t UDP
    ```

3. Back on the first terminal, run RoboFuzz (CONTAINER, terminal 1)
    Please note that each of the following substep should be run separately
    in a clean-slate RoboFuzz container (meaning that steps 1 and 2 must be
    preceded).

    a. Testing the offboard mode (via ROS trajectory setpoint)
    ```
    root@container_id:/robofuzz/src# ./fuzzer.py --px4-sitl-ros --method message --schedule sequence --repeat 1 --watchlist watchlist/px4.json --interval 0.02
    ```

    b. Testing via remote control commands (MAVLink)
    ```
    root@container_id:/robofuzz/src# ./fuzzer.py --px4-sitl-mav --method message --schedule sequence --seqlen 100 --repeat 1 --watchlist watchlist/px4.json --interval 0.1 --px4-flight-mode POSCTL
    ```

    You can test PX4 under different flight modes by switching the last
    parameter, i.e., `--px4-flight-mode POSCTL`. Please check the usage from
    `./fuzzer.py --help`.

    c. Testing by mutating parameter values (similar to PGFuzz)
    ```
    root@container_id:/robofuzz/src# ./fuzzer.py --px4-sitl-pgfuzz --method message --schedule single --repeat 1 --watchlist watchlist/px4.json --interval 15
    ```


## Reports and PoCs of the bugs RoboFuzz detected

Bug numbers (# columnn) in each table are in sync with Table 2 of the [paper](./paper.pdf).

### 1. PX4 bugs

| # | Fuzzing mode        | Short description                                                                 | Report/PoC                                                                |
|---|---------------------|-----------------------------------------------------------------------------------|---------------------------------------------------------------------------|
| 1 | `--px4-sitl-ros`    | `MPC_ACC_HOR_MAX` parameter violation                                             | https://github.com/PX4/PX4-Autopilot/issues/18033                         |
| 2 | `--px4-sitl-ros`    | Controller violates feed-forward setpoint specification                           | https://github.com/PX4/PX4-user_guide/issues/1458                         |
| 3 | `--px4-sitl-ros`    | Doc-implementation mismatch of trajectory setpoint message definition             | https://github.com/PX4/PX4-Autopilot/issues/18855#issuecomment-994922384  |
| 4 | `--px4-sitl-ros`    | Incorrect definitions of applicable parameters in offboard mode                   | https://github.com/PX4/PX4-Autopilot/issues/18855#issuecomment-1008432087 |
| 5 | `--px4-sitl-mav`    | Doc-implementation mismatch of `MPC_POS_MODE` parameter                           | https://github.com/PX4/PX4-Autopilot/issues/19101#issuecomment-1034174009 |
| 6 | `--px4-sitl-mav`    | Doc-implementation mismatch of `MPC_ACC_HOR` and `MPC_ACC_HOR_MAX` parameters     | https://github.com/PX4/PX4-Autopilot/issues/19101#issuecomment-1050916449 |
| 7 | `--px4-sitl-mav`    | Doc-implementation mismatch of `MPC_ACC_UP_MAX` and `MPC_ACC_DOWN_MAX` parameters | https://github.com/PX4/PX4-Autopilot/issues/19101#issuecomment-1034174009 |
| 8 | `--px4-sitl-pgfuzz` | Drone moves in horizontal direction in the Takeoff mode (spec violation)          | https://github.com/PX4/PX4-Autopilot/issues/19268                         |

### 2. TurtleBot3 bugs

| #  | Fuzzing mode                  | Short description                                             | Report/PoC                                                                  |
|----|-------------------------------|---------------------------------------------------------------|-----------------------------------------------------------------------------|
| 9  | `--tb3-sitl`                  | Doc-implementation mismatch of maximum velocity               | https://github.com/ROBOTIS-GIT/turtlebot3/issues/765                        |
| 10 | `--tb3-sitl`                  | Constraining logic fails to clamp velocity to the valid range | https://github.com/ROBOTIS-GIT/turtlebot3/issues/765#issuecomment-891092104 |
| 11 | `--tb3-hitl` and `--tb3-sitl` | Phy-sim discrepancy of maximum achievable torque              | https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/170            |
| 12 | `--tb3-hitl` and `--tb3-sitl` | Phy-sim discrepancy of maximum achievable velocity            | https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/170            |
| 13 | `--tb3-hitl` and `--tb3-sitl` | Phy-sim discrepancy in handling LiDAR scan data               | https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/178            |

### 3. Move It 2 + PANDA manipulator bugs

| #  | Fuzzing mode    | Short description                                  | Report/PoC                                                   |
|----|-----------------|----------------------------------------------------|--------------------------------------------------------------|
| 14 | `--test-moveit` | Doc-implementation mismatch of joint limits        | https://github.com/ros-planning/moveit_resources/issues/116  |
| 15 | `--test-moveit` | Velocities are zero when the manipulator is moving | https://github.com/ros-planning/moveit2_tutorials/issues/333 |

### 4. Turtlesim bugs

| #  | Fuzzing mode                                    | Short description                                | Report/PoC                                      |
|----|-------------------------------------------------|--------------------------------------------------|-------------------------------------------------|
| 16 | `--ros-pkg turtlesim --ros-node turtlesim_node` | Type confusion while normalizing an angle        | https://github.com/ros/ros_tutorials/issues/128 |
| 17 | `--ros-pkg turtlesim --ros-node turtlesim_node` | Missing validation of NaN/INF in input variables | https://github.com/ros/ros_tutorials/issues/129 |

### 5. ROSIDL bugs

| #  | Fuzzing mode    | Short description                                                                     | Report/PoC                                          |
|----|-----------------|---------------------------------------------------------------------------------------|-----------------------------------------------------|
| 18 | `--test-rosidl` | All floats are treated as doubles due to missing boundary checks                      | https://github.com/ros2/rosidl_python/pull/128      |
| 19 | `--test-rosidl` | Message setter does not handle byte correctly, treating them as string literals       | https://github.com/ros2/rosidl_runtime_py/issues/14 |
| 20 | `--test-rosidl` | Missing data range checks for int arrays                                              | https://github.com/ros2/rosidl_python/issues/153    |
| 21 | `--test-rosidl` | Missing data range checks for float arrays                                            | https://github.com/ros2/rosidl_python/issues/153    |
| 22 | `--test-rosidl` | Implicit type casting of array elements alters data without notifying                 | https://github.com/ros2/rosidl_python/issues/153    |
| 23 | `--test-rosidl` | Missing type checks for bool array elements, allowing data of any type to be stored   | https://github.com/ros2/rosidl_python/issues/153    |
| 24 | `--test-rosidl` | Missing type checks for byte array elements, allowing data of any type to be stored   | https://github.com/ros2/rosidl_python/issues/153    |
| 25 | `--test-rosidl` | Missing type checks for string array elements, allowing data of any type to be stored | https://github.com/ros2/rosidl_python/issues/153    |

### 6. RCL API bugs

| #  | Fuzzing mode                                                | Short description                                                                          | Report/PoC                                 |
|----|-------------------------------------------------------------|--------------------------------------------------------------------------------------------|--------------------------------------------|
| 26 | `--test-cli`                                                | `_on_parameter_events` always returns True, regardless of the result                       | https://github.com/ros2/rclpy/pull/817     |
| 27 | `--test-rcl --rcl-api publisher --rcl-job create_publisher` | Missing sanity checks for rmw handle in `rclpy_create_publisher`                           | https://github.com/ros2/rclpy/issues/826   |
| 28 | `--test-cli`                                                | `rclcpp` internally throws an exception that cannot be caught                              | https://github.com/ros2/rclcpp/issues/1581 |
| 29 | `--test-rcl --rcl-api node --rcl-job create_node`           | Node param event checks rely on subscription, violates design principle of param callbacks | https://github.com/ros2/rclcpp/issues/1758 |
| 30 | `--test-rcl --rcl-api node --rcl-job create_node`           | Error checking code is unreachable                                                         | https://github.com/ros2/rclcpp/issues/1758 |
