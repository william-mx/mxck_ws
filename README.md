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
