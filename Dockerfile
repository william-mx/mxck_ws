FROM ros:humble-ros-base-jammy

RUN apt update && apt install -y --no-install-recommends \
    ros-$ROS_DISTRO-librealsense2* \
    ros-$ROS_DISTRO-realsense2-* 

RUN mkdir /microros_ws \
 && cd /microros_ws \
 && git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup \
 && . /opt/ros/humble/setup.sh \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src -y \
 && colcon build \
 && . install/local_setup.sh \
 && ros2 run micro_ros_setup create_agent_ws.sh \
 && ros2 run micro_ros_setup build_agent.sh

RUN apt install -y ros-humble-rplidar-ros

RUN mkdir -p /vesc_ws \
 && git clone -b ros2 https://github.com/f1tenth/vesc.git /vesc_ws/src \
 && cd /vesc_ws/src \
 && rm -r ./vesc_ackermann && rm -r ./vesc \
 && git clone -b humble https://github.com/ros-drivers/transport_drivers.git \
 && cd /vesc_ws \
 && rosdep update && rosdep install --from-paths src -i -y \
 && . /opt/ros/humble/setup.sh \
 && colcon build

# https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/
RUN apt install -y python3-pip \
&& python3 -m pip install setuptools==58.2.0

COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN echo 'source /ros_entrypoint.sh' >> ~/.bashrc

COPY ./autorun.sh /
ENTRYPOINT ["./autorun.sh"]
CMD ["false"]

WORKDIR ./humble_ws