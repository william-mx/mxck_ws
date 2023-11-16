# MXCarkit ROS Workspace

Clone this repository.
```
git clone -b mxck_base https://github.com/william-mx/mxck_ws.git ~/mxck_ws/mxck_base
```

Make shell scripts and python files executable.
```
cd ~/mxck_ws/mxck_base
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

Open another terminal and connect to the docker container.
```
./add_ros_docker.sh
```

## Visualize ROS data
[Foxglove](https://foxglove.dev/ros) is a great tool to visualize your ROS data. The data can be viewed either locally on the Jetson or on any other computer as long it is connected to the same network.

Foxglove Studio can be run as a standalone desktop app or be accessed via your browser. At this point in time (11/2023), the desktop version is not supported on Jetsons Arm64 architecture, so we have to use the [Web Console](https://studio.foxglove.dev). Foxglove Studio currently requires Chrome v76+ so make sure you have the Chromium web browser installed ```sudo apt-get install chromium-browser```. Alternatively, the data can be visualized remotely on a second computer either via the [Web Console](https://studio.foxglove.dev) or via the [Desktop Version](https://foxglove.dev/download). **(recommended)**

All sensors and also the motors can be controlled via the mxck_run.launch file in the mxck_run package. The individual sensors can be controlled via command line arguments. Have a look at the launch file in order to understand the whole procedure. By default, all sensors are switched off. Let's start the camera and watch the video stream in foxglove.

```
roslaunch mxck_run mxck_run.launch run_camera:=true run_foxglove:=false
```
When running the Foxglove node, a UDP port is passed as an argument. By default this port has the value ```8765``` but you can also specify a different value. On the Jetson, we simply open the [Web Console](https://studio.foxglove.dev), click open connection, and enter ```ws://localhost:8765``` as the WebSocket URL.

On another computer, we first need to make sure that the Jeston and the computer are on the same Wi-Fi network. Then we have to find out the IP address of the Jetson (e.g. 192.168.178.39) using ```ifconfig```. Then we open Foxglove Studio on our computer, click on open connection, and enter ```ws://192.168.178.39:8765``` as the WebSocket URL.
