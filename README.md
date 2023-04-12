# MXCarkit ROS Workspace

Clone this repository.
```
git clone --recurse-submodules https://github.com/william-mx/mxck_ws.git
```

Make shell scripts and python files executable.
```
cd ./mxck_ws
find . -type f -name '*.py' -o -name '*.sh' -exec chmod +x {} \;
```

Run docker image
```
./run_ros_docker.sh
```


Build the workspace
```
catkin_make
source devel/setup.bash
```
