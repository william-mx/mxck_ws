# FROM mxwilliam/mxck:mxck-foxy-pytorch-l4t-35.4
FROM mxwilliam/mxck:mxck-humble-ubuntu-22.04

# Upgrade pip and install Python packages
# RUN python3 -m pip install \
# ...

# Update system and install ROS packages
# RUN apt update \
# && apt install --yes \
# ...

RUN python3 -m pip install --no-cache-dir git+https://github.com/william-mx/ros2_numpy.git

COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN echo 'source /ros_entrypoint.sh' >> ~/.bashrc

COPY ./autorun.sh /
ENTRYPOINT ["./autorun.sh"]
CMD ["false"]

COPY ./.bash_aliases /root/.bash_aliases