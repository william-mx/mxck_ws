# .bash_aliases

# Start Foxglove + TF broadcast
alias run_foxglove='ros2 launch mxck_run mxck_run_launch.py run_foxglove:=true broadcast_tf:=true'

# Manual vehicle control (e.g. rc control)
alias kickstart='ros2 launch vehicle_control manual_control_launch.py'

# Start micro-ROS node
alias run_micro='ros2 launch mxck_run mxck_run_launch.py run_micro:=true'

# RGB camera only
alias run_camera='ros2 launch mxck_run realsense_launch.py camera:=true'

# RGB-D mode
alias run_rgbd='ros2 launch mxck_run realsense_launch.py camera:=true rgbd:=true'

# IMU only (accelerometer + gyroscope)
alias run_imu='ros2 launch mxck_run realsense_launch.py rs_imu:=true'

# Left IR camera only
alias run_ir_left='ros2 launch mxck_run realsense_launch.py ir_left:=true'

# Right IR camera only
alias run_ir_right='ros2 launch mxck_run realsense_launch.py ir_right:=true'

# IR projector emitter
alias run_projector='ros2 launch mxck_run realsense_launch.py ir_projector:=true'

# Camera + IMU only — for visual-inertial odometry (VIO) pipelines like ORB-SLAM, etc.
alias run_vio='ros2 launch mxck_run realsense_launch.py camera:=true rs_imu:=true'

