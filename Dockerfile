FROM dustynv/ros:melodic-ros-base-l4t-r32.4.4

RUN apt-get update \
 && apt-get install --yes \
    ros-$ROS_DISTRO-tf \
    ros-$ROS_DISTRO-serial \
    ros-$ROS_DISTRO-rosserial \
    ros-$ROS_DISTRO-rosserial-python \
    ros-$ROS_DISTRO-rplidar-ros \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-ackermann-msgs \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-roslint \
    ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-foxglove-msgs \
    python-scipy


RUN git clone https://github.com/f1tenth/vesc.git /vesc_ws/src \
 && cd /vesc_ws \
 && rm -r src/vesc src/vesc_ackermann \
 && . /opt/ros/melodic/setup.sh \
 && catkin_make

# Utilizing Python 3 for integrating pyrealsense2 with pyzmq, enabling efficient image streaming.
# This setup significantly outperforms traditional ROS image transport methods in speed.

RUN apt update \
 && apt install --yes \
 python3.8 \
 python3-pip \
 && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.6 1 \
 && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 2

RUN python3 -m pip install --upgrade pip \
    && python3 -m pip install \
    pyrealsense2 \
    numpy \
    pyzmq \
    pyyaml \
    rospkg \
    opencv-python

COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN echo 'source /ros_entrypoint.sh' >> ~/.bashrc

COPY ./autorun.sh /
ENTRYPOINT ["./autorun.sh"]
CMD ["false"]

WORKDIR ./melodic_ws



