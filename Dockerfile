FROM mxwilliam/mxck:mxck-foxy-pytorch-l4t-35.4
# FROM mxwilliam/mxck:mxck-humble-base-l4t-36.4.0

# Upgrade pip and install Python packages
# RUN python3 -m pip install \
# ...

# Update system and install ROS packages
# RUN apt update \
# && apt install --yes \
# ...

COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN echo 'source /ros_entrypoint.sh' >> ~/.bashrc

COPY ./autorun.sh /
ENTRYPOINT ["./autorun.sh"]
CMD ["false"]