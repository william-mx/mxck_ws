#!/bin/bash

# set the permissions the X server host
# see http://wiki.ros.org/docker/Tutorials/GUI
xhost +local:docker

BASE_DIR=/home/mxck/mxck_ws/mxck_base

# run ros docker container
sudo docker run -it --rm \
--runtime nvidia \
--mount type=bind,source=$BASE_DIR,target=/melodic_ws \
--mount type=bind,source=$BASE_DIR/ros_entrypoint.sh,target=/ros_entrypoint.sh \
--mount type=bind,source=/dev,target=/dev \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--privileged \
--net=host \
--name mxck_base_melodic \
mxck_base_melodic:latest $1











