# MXCarkit ROS Perception

The mxck_perception workspace is based on jetson-inference, which is provided by Nvidia and is the ideal platform for Deploying Deep Learning on NVIDIA Jetson devices. Have a look at their [repository](https://github.com/dusty-nv/jetson-inference), there are many good tutorials (e.g. how to run a custom object detection on the jetson).

First, we follow the official [instructions](https://github.com/dusty-nv/jetson-inference/blob/master/docs/aux-docker.md) and set up the jetson inference container with ros noetic support by running the following commands.

```
cd /home/mxck
git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference
cd jetson-inference
docker/run.sh --ros=noetic
```

Our docker image is based on the docker image that has just been installed. Open a new terminal and check the name with the command ```sudo docker ps```. Normally it should be called ```dustynv/ros:noetic-pytorch-l4t-r35.2.1``` but it may differ depending on your installed jetpack version. Remember the name of the docker image, you will need it later.

Next, we can clone this repository. It is important that the mxck_base workspace is set up first. If this has not yet been done, follow the instructions [here](https://github.com/william-mx/mxck_ws). When this is done, execute the following command.

```
git clone -b mxck_perception https://github.com/william-mx/mxck_ws.git ~/mxck_ws/mxck_perception
```

Make shell scripts and python files executable.
```
cd ~/mxck_ws/mxck_base
sudo find . -type f -name '*.py' -o -name '*.sh' -exec chmod +x {} \;
sudo find . -type f -name '*.py' -exec dos2unix {} \;
```

Before you build the docker image, check the first line of the ```Dockerfile```. It should contain the name of the docker image we have just built.
If the name is correct then execute the following command and install the docker image.
```
cd ~/mxck_ws/mxck_perception
sudo docker build -t mxck_perception_noetic .
```

Run docker image.
```
./run_ros_docker.sh
```

Open another terminal and connect to the docker container.
```
./add_ros_docker.sh
```
