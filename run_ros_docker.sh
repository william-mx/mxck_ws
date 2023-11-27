#!/bin/bash


# set the permissions the X server host
# see http://wiki.ros.org/docker/Tutorials/GUI
xhost +local:docker

# paths to some project directories
BASE_DIR="/home/mxck/mxck_ws/mxck_perception"

NV_DIR=/home/mxck/jetson-inference

NETWORKS_DIR="data/networks"
CLASSIFY_DIR="python/training/classification"
DETECTION_DIR="python/training/detection/ssd"
RECOGNIZER_DIR="python/www/recognizer"

DOCKER_ROOT="/jetson-inference" # jetson-inference base

# generate mount commands
DATA_VOLUME=" \
--volume $NV_DIR:$DOCKER_ROOT \
--volume $NV_DIR/data:$DOCKER_ROOT/data \
--volume $NV_DIR/$CLASSIFY_DIR/data:$DOCKER_ROOT/$CLASSIFY_DIR/data \
--volume $NV_DIR/$CLASSIFY_DIR/models:$DOCKER_ROOT/$CLASSIFY_DIR/models \
--volume $NV_DIR/$DETECTION_DIR/data:$DOCKER_ROOT/$DETECTION_DIR/data \
--volume $NV_DIR/$DETECTION_DIR/models:$DOCKER_ROOT/$DETECTION_DIR/models \
--volume $NV_DIR/$RECOGNIZER_DIR/data:$DOCKER_ROOT/$RECOGNIZER_DIR/data "

# run ros docker container
sudo docker run -it --rm \
--runtime nvidia \
--mount type=bind,source=$BASE_DIR,target=/noetic_ws \
--mount type=bind,source=$BASE_DIR/ros_entrypoint.sh,target=/ros_entrypoint.sh \
--mount type=bind,source=/dev,target=/dev \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
-v /tmp/argus_socket:/tmp/argus_socket \
-v /etc/enctune.conf:/etc/enctune.conf \
-v /etc/nv_tegra_release:/etc/nv_tegra_release \
-v /tmp/nv_jetson_model:/tmp/nv_jetson_model \
$DATA_VOLUME \
--privileged \
--net host \
--name mxck_perception_noetic \
mxck_perception_noetic:latest $1









