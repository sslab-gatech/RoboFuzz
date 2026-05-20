# moveit2 humble target
RoboFuzz was developed for ROS2 foxy. On ROS2 humble, moveit2 and ros-planning
seems to have changed in a way it is not backward-compatible.

This branch contains the test harness for moveit2-humble.

## Setting Up
```sh
cd /PATH/TO/ws_moveit2/src/moveit2_tutorials/doc/tutorials
ros2 pkg create --build-type ament_cmake --dependencies moveit_ros_planning_interface rclcpp --node-name moveit2_harness moveit2_harness
cp moveit2_harness.cpp /PATH/TO/ws_moveit2/src/moveit2_tutorials/doc/tutorials/moveit2_harness/src
```

Add build target to `/PATH/TO/ws_moveit2/src/moveit2_tutorials/CMakeLists.txt`:
```
add_subdirectory(doc/tutorials/moveit2_harness)
```

Build:
```
cd /PATH/TO/ws_moveit2/
colcon build --mixin release
```

Test:
```
source
ros2 run moveit2_harness moveit2_harness 0.2 0.2 0.2 0.2
```

## Instrumentation
Build mclangwrapper of rofer-humble:
```
cd /home/seulbae/workspace
git clone git@github.com:postech-compsec/rofer-humble.git
cd rofer-humble/fuzz_manager
ROFER_INSTRUMENT=1 ./build.sh
cd ..
ls mutator/build/bin | grep mclang
```

Instrument using mclangwrapper:
```
colcon build \
  --mixin release \
  --cmake-clean-cache \
  --cmake-args \
    -DCMAKE_C_COMPILER=/home/seulbae/workspace/rofer-humble/mutator/build/bin/mclangwrapper \
    -DCMAKE_CXX_COMPILER=/home/seulbae/workspace/rofer-humble/mutator/build/bin/mclangwrapper++
```

