#!/bin/bash

# https://answers.ros.org/question/205976/sourcing-from-multiple-workspaces/
# build vesc_ws first 
# then overlay melodic_ws

# Overlaying ROS Workspaces
MELODIC_SH=/opt/ros/melodic/setup.bash
VESC_SH=/vesc_ws/devel/setup.bash
MXCK_SH=/melodic_ws/devel/setup.bash

# setup ros environment
source $MELODIC_SH
echo "sourcing $MELODIC_SH"

# setup vesc environment
source $VESC_SH
echo "sourcing $VESC_SH"

# setup mxck environment
if test -f "$MXCK_SH"; then
    source $MXCK_SH
    echo "sourcing $MXCK_SH"
fi

exec "$@"


