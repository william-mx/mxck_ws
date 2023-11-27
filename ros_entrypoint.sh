#!/bin/bash

NOETIC_SH=/opt/ros/noetic/setup.bash
MXCK_SH=/noetic_ws/devel/setup.bash

# setup ros environment
source $NOETIC_SH
echo "sourcing $NOETIC_SH"

# build mxck environment
if test ! -f "$MXCK_SH"; then
    catkin_make
fi

# setup mxck environment
source $MXCK_SH
echo "sourcing $MXCK_SH"

    
exec "$@"


