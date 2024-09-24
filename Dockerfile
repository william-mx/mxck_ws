FROM mxwilliam/mxck:mxck-noetic-perception-l4t-35.2.1

# Upgrade pip and install Python packages
RUN python3 -m pip install \
    gdown

# Update system and install ROS packages
# RUN apt update \
# && apt install --yes \
# ...

# Replace ros_entrypoint.sh
COPY ./ros_entrypoint.sh /ros_entrypoint.sh

# Copy autorun script
COPY ./autorun.sh /

# Set the entrypoint and default command
ENTRYPOINT ["/autorun.sh"]
CMD ["false"]

# Set working directory
WORKDIR /noetic_ws
