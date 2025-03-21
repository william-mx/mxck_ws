# MXCarkit ROS2 Humble Workspace

In theory, it is best practice to use the official ROS Docker images from Nvidia, available at [https://github.com/dusty-nv/jetson-containers/tree/master](https://github.com/dusty-nv/jetson-containers/tree/master). While the installation works and ROS runs stably, installing additional packages can be challenging. Therefore, we instead used the official Docker image from ROS itself, available at [https://hub.docker.com/_/ros/](https://hub.docker.com/_/ros/), and have successfully tested it on JetPack 5.1.

Clone this repository.
```
git clone -b mxck2_ws https://github.com/william-mx/mxck_ws.git ~/mxck2_ws
```

Make shell scripts and python files executable.
```
cd ~/mxck2_ws
sudo find . -type f -name '*.py' -o -name '*.sh' -exec chmod +x {} \;
sudo find . -type f -name '*.py' -exec dos2unix {} \;
```

Build docker image.
```
sudo docker build -t mxck2_humble .
```

