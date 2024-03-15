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
The *roscore* is also started immediately when the Docker container is running.
In *ros_entrypoint.sh* you can also configure that certain sensors or programs start automatically, for example the camera or foxglove.
```
./run_ros_docker.sh
```

Open another terminal and connect to the docker container.
```
./add_ros_docker.sh
```


## Automating MXCarKit Setup: Quick Start and Configuration Guide

We've made a script called `startup_mxck.sh`. The `startup_mxck.sh` script is located in our repository at [https://github.com/william-mx/MXcarkit](https://github.com/william-mx/MXcarkit) and is automatically copied to the `/home/mxck/mxck_ws` directory when the `initial_setup.sh` script is executed. If for some reason this doesn't happen, you should manually copy it to the directory. It helps your computer connect to the MXCarKit right away when it starts. This script sets up a hotspot for connecting to the car kit and starts basic programs like Foxglove to stream data live. This script contains two commands:

`sudo nmcli dev wifi hotspot ifname wlan0 ssid mxck0000 password mxck0000`: This command sets up a hotspot on the MXCarKit with the SSID `mxck0000` and password `mxck0000`. You'll need to customize the ID (e.g., change `0000` to match your specific MXcarKit ID, such as `0016`). The default IP address of the MXCarKit when running as a hotspot is `10.42.0.1`. Once connected to the hotspot, the car kit will be visible at this IP address.

The second command opens a new terminal and executes the `/home/mxck/mxck_ws/mxck_base/run_ros_docker.sh 'true'` command. The `run_ros_docker.sh` script initiates a ROS Docker container within our MXcarKit's mxck_base workspace. The 'true' flag tells the script that an Entrypoint shell script should be executed when the container is launched.

To ensure the `startup_mxck.sh` script executes immediately when the car kit starts, follow these three steps:

1. **Search for the Startup Application Tool on your Jetson.** Once found, add a new entry with a meaningful name like `startup mxck`. In the command field, enter `/home/mxck/startup_mxck.sh` to specify the script that should run at startup.

2. **Enable automatic login for the mxck user.** This step is crucial because if the mxck user is not logged in automatically, the script will only start after manual login. Configuring automatic login allows Ubuntu to bypass the password prompt for the mxck user, ensuring the script runs seamlessly at boot.

3. **Modify the visudo configuration for password-less sudo access.** Without this, executing the script as a sudo user would require a password, introducing another manual step we aim to eliminate for automation. To edit the visudo config, use the command `sudo gedit /etc/sudoers`. At the end of the file, add the following line to grant the mxck user sudo access without a password:

    ```
    mxck ALL=(ALL) NOPASSWD: ALL
    ```

This line allows the mxck user to execute any command as sudo without being prompted for a password, ensuring the script can run automatically.


Before rebooting the Jetson and testing our `startup_mxck.sh` script, we want to understand which ROS nodes run when we start the `run_ros_docker.sh` script with the `true` argument. First, we look at the Dockerfile in our repository [https://github.com/william-mx/mxck_ws](https://github.com/william-mx/mxck_ws). These three lines in the Dockerfile:

```
COPY ./autorun.sh /
ENTRYPOINT ["./autorun.sh"]
CMD ["false"]
```

This instructs the Docker container to start the `autorun.sh` script upon initialization, passing `false` as the default argument to the `autorun.sh` script.

Next, we examine the `autorun.sh` file in our repository. At the beginning of this script, it sources our ROS workspace. The most relevant part for us is the last code block:

```
# run command
if [ "$1" = "true" ];
then
  roslaunch mxck_run mxck_run.launch run_micro:=true run_foxglove:=true run_camera:=true run_motors:=false
else
  bash
fi;
```

When we pass `true` to the script, it will automatically execute a `roslaunch` command when the Docker container starts, launching specific ROS nodes. If any other value is passed (or by default, `false`), it will simply open a bash terminal, allowing us to manually interact with the container.

In the `mxck_run.launch` file, the arguments `run_micro:=true`, `run_foxglove:=true`, `run_camera:=true`, and `run_motors:=false` control various aspects of the MXCarKit's startup behavior:

- **`run_micro:=true`** activates rosserial communication with the microcontroller, enabling sensor data from devices like IMUs and ultrasonic sensors to be published to ROS topics.

- **`run_foxglove:=true`** starts data streaming to Foxglove Studio, allowing for real-time visualization of sensor data and camera feeds.

- **`run_camera:=true`** turns on the camera for live video streaming.

- **`run_motors:=false`** keeps the motors off by default for safety, preventing the car kit from moving unexpectedly at startup.

Each argument toggles a specific feature of the car kit, directly affecting what capabilities are available immediately after the system boots.


Besides the discussed arguments, there are many more available for configuring the startup behavior of the MXCarKit. To explore all possible options, you can inspect the `mxck_base/src/mxck_run/launch/mxck_run.launch` file. This file contains detailed configurations for various features and components.


## Visualize ROS data
[Foxglove](https://foxglove.dev/ros) is a great tool to visualize your ROS data. The data can be viewed either locally on the Jetson or on any other computer as long it is connected to the same network.

Foxglove Studio can be run as a standalone desktop app or be accessed via your browser. At this point in time (11/2023), the desktop version is not supported on Jetsons Arm64 architecture, so we have to use the [Web Console](https://studio.foxglove.dev). Foxglove Studio currently requires Chrome v76+ so make sure you have the Chromium web browser installed ```sudo apt-get install chromium-browser```. Alternatively, the data can be visualized remotely on a second computer either via the [Web Console](https://studio.foxglove.dev) or via the [Desktop Version](https://foxglove.dev/download). **(recommended)**

All sensors and also the motors can be controlled via the mxck_run.launch file in the mxck_run package. The individual sensors can be controlled via command line arguments. Have a look at the launch file in order to understand the whole procedure. By default, all sensors are switched off. Let's start the camera and watch the video stream in foxglove.

```
roslaunch mxck_run mxck_run.launch run_camera:=true run_foxglove:=false
```
When running the Foxglove node, a UDP port is passed as an argument. By default this port has the value ```8765``` but you can also specify a different value. On the Jetson, we simply open the [Web Console](https://studio.foxglove.dev), click open connection, and enter ```ws://localhost:8765``` as the WebSocket URL.

On another computer, we first need to make sure that the Jeston and the computer are on the same Wi-Fi network. Then we have to find out the IP address of the Jetson (e.g. 192.168.178.39) using ```ifconfig```. Then we open Foxglove Studio on our computer, click on open connection, and enter ```ws://192.168.178.39:8765``` as the WebSocket URL.
