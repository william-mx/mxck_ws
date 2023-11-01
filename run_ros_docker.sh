#!/bin/bash

# set the permissions the X server host
# see http://wiki.ros.org/docker/Tutorials/GUI
xhost +local:docker

# run ros docker container
sudo docker run -it --rm \
--mount type=bind,source=/home/mxck/tmp2/mxck_base,target=/melodic_ws \
--mount type=bind,source=/dev,target=/dev \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--privileged \
--net=host \
--entrypoint bash \
--name mxck_base_melodic \
mxck_base_melodic:latest



 










