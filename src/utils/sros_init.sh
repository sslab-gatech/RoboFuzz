#!/bin/bash

ros2 security create_keystore fuzzing_keys
ros2 security create_key fuzzing_keys /fuzzer/_fuzzer
ros2 security create_key fuzzing_keys /fuzzer/sros2_node

ros2 security create_permission fuzzing_keys /fuzzer/_fuzzer policies/fuzzer.policy.xml
