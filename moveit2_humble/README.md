# moveit2 humble target
RoboFuzz was developed for ROS2 foxy. On ROS2 humble, moveit2 and ros-planning
seems to have changed in a way it is not backward-compatible.

This branch contains the test harness for moveit2-humble.

## Setting Up
(pwd: `~/robofuzz/moveit2_humble`)
```sh
export WD=$(pwd)
cd /PATH/TO/ws_moveit2/src/moveit2_tutorials/doc/tutorials
ros2 pkg create --build-type ament_cmake --dependencies moveit_ros_planning_interface rclcpp --node-name moveit2_harness moveit2_harness
cp $(WD)/moveit2_harness.cpp moveit2_harness/src/
```

Open `/PATH/TO/ws_moveit2/src/moveit2_tutorials/CMakeLists.txt` and add the
following line before `ament_export_dependencies`:
```
add_subdirectory(doc/tutorials/moveit2_harness)
```

Build:
```
cd /PATH/TO/ws_moveit2/
colcon build --mixin release
```

Test (terminal 1 - rviz and robot):
```
source /opt/ros/humble/setup.zsh # or your own ros env
source /PATH/TO/ws_moveit2/install/setup.zsh
ros2 launch moveit2_tutorials demo.launch.py
```

Test (terminal 2 - commander):
```
source /opt/ros/humble/setup.zsh # or your own ros env
source /PATH/TO/ws_moveit2/install/setup.zsh
ros2 run moveit2_harness moveit2_harness 0.2 0.2 0.2 0.2
```

If successful, follow the steps below to rebuild with rofer's instrumentation.

## Instrumentation
Build mclangwrapper of rofer-humble:
```
cd $WD
git clone git@github.com:postech-compsec/rofer-humble.git
cd rofer-humble/fuzz_manager
ROFER_INSTRUMENT=1 ./build.sh
cd ..
ls mutator/build/bin | grep mclang
```

Instrument moveit2 using mclangwrapper:
```
cd /PATH/TO/ws_moveit2
colcon build \
  --mixin release \
  --cmake-clean-cache \
  --cmake-args \
    -DCMAKE_C_COMPILER=$WD/rofer-humble/mutator/build/bin/mclangwrapper \
    -DCMAKE_CXX_COMPILER=$WD/rofer-humble/mutator/build/bin/mclangwrapper++
```

