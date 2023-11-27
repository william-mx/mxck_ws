FROM dustynv/ros:noetic-pytorch-l4t-r35.2.1

RUN apt purge --yes '*opencv*'

RUN apt update \
 && apt install --yes \
    ros-$ROS_DISTRO-ackermann-msgs \
    ros-$ROS_DISTRO-foxglove-msgs \
    ros-$ROS_DISTRO-vision-opencv \
    ros-$ROS_DISTRO-rqt-reconfigure

RUN apt update \
 && python3 -m pip install \
 pyrealsense2 \
 matplotlib

# the shell script ros_entrypoint.sh is already added to ~/.bashrc in base image
# so we just have to replace the file
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
# RUN echo 'source /ros_entrypoint.sh' >> ~/.bashrc

COPY ./autorun.sh /
ENTRYPOINT ["./autorun.sh"]
CMD ["false"]

WORKDIR ./noetic_ws



