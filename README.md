# MXCarkit ROS Workspace

Clone this repository.
```
git clone -b mxck_base https://github.com/william-mx/mxck_ws.git ./mxck_base
```

Make shell scripts and python files executable.
```
cd ./mxck_base
sudo find . -type f -name '*.py' -o -name '*.sh' -exec chmod +x {} \;
sudo find . -type f -name '*.py' -exec dos2unix {} \;
```

Build docker image.
```
sudo docker build -t mxck_base_melodic .
```

Run docker image.
```
./run_ros_docker.sh
```

Build the workspace.
```
catkin_make
source devel/setup.bash
```

## Visualize ROS data
[Foxglove](https://foxglove.dev/ros) is a great tool to visualize your ROS data. The data can be viewed either locally on the Jetson or on any other computer as long it is connected to the same network.

Foxglove Studio can be run as a standalone desktop app or be accessed via your browser. At this point in time (11/2023), the desktop version is not supported on Jetsons Arm64 architecture, so we have to use the [Web Console](https://studio.foxglove.dev). Foxglove Studio currently requires Chrome v76+ so make sure you have the Chromium web browser installed ```sudo apt-get install chromium-browser```. Alternatively, the data can be visualized remotely on a second computer either via the [Web Console](https://studio.foxglove.dev) or via the [Desktop Version](https://foxglove.dev/download). **(recommended)**

