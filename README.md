# MXCarkit ROS Workspace

Clone this repository.
```
git clone --recurse-submodules https://github.com/william-mx/mxck_ws.git
```

Make shell script executable.
```
cd ./mxck_ws
chmod +x run_ros_docker.sh add_ros_docker.sh 
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