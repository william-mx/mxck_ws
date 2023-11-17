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

# build mxck environment
if test ! -f "$MXCK_SH"; then
    catkin_make
fi

# setup mxck environment
source $MXCK_SH
echo "sourcing $MXCK_SH"

if [ "$1" = "true" ];
then
  roslaunch mxck_run mxck_run.launch \
  run_camera:=false \
  run_lidar:=false \
  run_micro:=false \
  run_pdc:=false \
  run_imu:=false \
  run_foxglove:=false
fi;
    
exec "$@"


