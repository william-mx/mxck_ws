# MXCarkit ROS Workspace

Clone this repository.
```
git clone --recurse-submodules https://github.com/william-mx/mxck_ws.git
```


Run docker image
```
./run_ros_docker.sh
```

We need to build the realsense_camera package from source in order to receive the imu data. To do so, we follow the official installation instructions from [realsense](http://wiki.ros.org/realsense_camera/Tutorials/Building_from_Sources). More about that issue can be found [here](https://github.com/IntelRealSense/librealsense/issues/10304).

``` python
# Navigate to the ros package
cd ./src/realsense

# Checkout the latest stable code
git checkout -b stable-branch stable

# Update the dependencies database
rosdep update

# Attempt to install dependencies
# Requires that your login has sudo privileges to instal as root using apt-get
cd ~/catkin_ws
rosdep -y install --from-paths src --ignore-src


# Build the package
catkin_make

```