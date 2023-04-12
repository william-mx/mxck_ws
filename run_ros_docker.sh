#!/bin/bash

# set the permissions the X server host
# see http://wiki.ros.org/docker/Tutorials/GUI
xhost +local:docker

# run ros docker container
sudo docker run -it --rm \
--mount type=bind,source=/home/mxck/mxck_ws,target=/catkin_ws \
--mount type=bind,source=/dev,target=/dev \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--privileged \
--net=host \
--name ros_melodic_docker \
mxck_ros_hw:latest bash



 










