#!/bin/bash
colcon build
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/ros2_ws/install/setup.bash
ros2 run nlu nlu_node
exec "$@"

