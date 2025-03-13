#!/bin/bash

# set the permissions the X server host
# see http://wiki.ros.org/docker/Tutorials/GUI
xhost +local:docker

# run ros docker container
sudo docker run -it --rm \
--mount type=bind,source=/home/mxck/mxck2_ws,target=/mxck2_ws \
--mount type=bind,source=/dev,target=/dev \
--env="ROS_ROOT=/opt/ros/humble" \
--privileged \
--net=host \
--name mxck2_humble \
mxck2_humble bash








