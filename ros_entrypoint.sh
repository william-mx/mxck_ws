#!/bin/bash

# Overlaying ROS Workspaces
# ROS2_SH=/opt/ros/foxy/setup.bash
ROS2_SH=/opt/ros/humble/setup.bash
MICROROS_SH=/microros_ws/install/setup.bash
VESC_SH=/vesc_ws/install/setup.bash
RPLIDAR_SH=/rplidar_ws/install/setup.bash
MXCK_SH=/mxck2_ws/install/setup.bash


# setup ros2 environment
source $ROS2_SH
echo "sourcing $ROS2_SH"

# setup micro-ros environment
source $MICROROS_SH
echo "sourcing $MICROROS_SH"

# setup vesc environment
source $VESC_SH
echo "sourcing $VESC_SH"

# setup rplidar environment
source $RPLIDAR_SH
echo "sourcing $RPLIDAR_SH"

# build mxck environment
if test ! -f "$MXCK_SH"; then
    colcon build --symlink-install
fi

# setup mxck environment
source $MXCK_SH
echo "sourcing $MXCK_SH"
    
exec "$@"
