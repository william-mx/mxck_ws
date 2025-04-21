# .bash_aliases
alias run_foxglove='ros2 launch mxck_run mxck_run_launch.py run_foxglove:=true broadcast_tf:=true'
alias kickstart='ros2 launch vehicle_control manual_control_launch.py'
alias run_camera='ros2 launch mxck_run mxck_run_launch.py run_camera:=true'
alias run_micro='ros2 launch mxck_run mxck_run_launch.py run_micro:=true'
alias kill_ros='pkill -SIGINT -f -- "--ros-args"'
