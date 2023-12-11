# Gazebo Simulation MXCarkit
## Installation
Install ROS Noetic on your System:
http://wiki.ros.org/noetic/Installation
The full desktop installation already includes the correct gazebo Version.
You need to install gazebo 11

Clone the mxck_gazebo branch of this github to mxck_ws/mxck_gazebo folder:
```
git clone -b mxck_gazebo https://github.com/william-mx/mxck_ws.git ~/mxck_ws/mxck_gazebo
```

Go to the clone workspace:
```
cd ~/mxck_ws/mxck_gazebo
```
Build the workspace with catikin:
```
catkin_make
```

To load the model of the track you have to add the model path to the Gazebo model path. You can do that by adding the following to lines to your ~/.bashrc file.
```
source /usr/share/gazebo-11/setup.sh
export GAZEBO_MODEL_PATH=~/mxck_ws/mxck_gazebo/src/mxcarkit_description/worlds/models:${GAZEBO_MODEL_PATH}
```

## Starting the Simulation
Source the setup.bash:
```
source mxck_ws/mxck_gazebo/devel/setup.bash
```

Execute the launch file:
```
roslaunch mxcarkit_description mxcarkit_gazebo.launch
```
