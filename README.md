# MXCarkit ROS Workspace

Clone this repository.
```
git clone --recurse-submodules https://github.com/william-mx/mxck_ws.git
cd ./mxck_ws
git checkout dev
git submodule update --recursive
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

Check out branch

```
cd /catkin_ws/src/realsense-ros/ &&\
git checkout 2.3.1 &&\
sed -i 's/\ find_if/\ std::find_if/g' ./realsense2_camera/src/base_realsense_node.cpp
```

Build the workspace
```
cd /catkin_ws/src &&\
catkin_init_workspace &&\
cd .. &&\
rosdep install --from-paths src --ignore-src -r -y &&\
catkin_make clean &&\
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release &&\
catkin_make install &&\
source devel/setup.bash
```