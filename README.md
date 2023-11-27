# MXCarkit ROS Perception

The mxck_perception workspace is based on jetson-inference, which is provided by Nvidia and is the ideal platform for Deploying Deep Learning on NVIDIA Jetson devices. Have a look at their [repository](https://github.com/dusty-nv/jetson-inference), there are many good tutorials (e.g. how to run a custom object detection on the jetson).

So the first thing you should do is clone jetson-inference. For the latest instructions please follow the official [repository](https://github.com/dusty-nv/jetson-inference/blob/master/docs/aux-docker.md). By executing ```run.sh``` the docker container is set up with ros noetic support. You can close the docker container again by entering ```exit``` in the command line.

```
cd /home/mxck
git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference
cd jetson-inference
docker/run.sh --ros=noetic
```
Next, we can clone this repository. It is important that the mxck_base workspace is set up first. If this has not yet been done, follow the instructions [here](https://github.com/william-mx/mxck_ws).

```
git clone -b mxck_perception https://github.com/william-mx/mxck_ws.git ~/mxck_ws/mxck_perception
```

Make shell scripts and python files executable.
```
cd ~/mxck_ws/mxck_base
sudo find . -type f -name '*.py' -o -name '*.sh' -exec chmod +x {} \;
sudo find . -type f -name '*.py' -exec dos2unix {} \;
```

Build docker image.
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
