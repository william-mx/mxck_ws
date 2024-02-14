#!/bin/bash

# set the permissions the X server host
# see http://wiki.ros.org/docker/Tutorials/GUI
xhost +local:docker

# run ros docker container
sudo docker run -it --rm \
--mount type=bind,source=/home/mxck/mxck2_ws/mxck2_base,target=/humble_ws \
--mount type=bind,source=/dev,target=/dev \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="ROS_ROOT=/opt/ros/humble" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--privileged \
--name mxck2_base_humble \
mxck2_base_humble bash








