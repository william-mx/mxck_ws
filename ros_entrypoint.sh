#!/bin/bash

# Overlaying ROS Workspaces
HUMBLE_SH=/opt/ros/humble/setup.bash
MICROROS_SH=/microros_ws/install/setup.bash
VESC_SH=/vesc_ws/install/setup.bash
MXCK_SH=/humble_ws/install/setup.bash


# setup ros2 environment
source $HUMBLE_SH
echo "sourcing $HUMBLE_SH"

# setup micro-ros environment
source $MICROROS_SH
echo "sourcing $MICROROS_SH"

# setup vesc environment
source $VESC_SH
echo "sourcing $VESC_SH"

# build mxck environment
if test ! -f "$MXCK_SH"; then
    colcon build --symlink-install
fi

# setup mxck environment
source $MXCK_SH
echo "sourcing $MXCK_SH"
    
exec "$@"
