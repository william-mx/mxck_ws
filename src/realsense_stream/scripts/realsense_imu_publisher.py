#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import pyrealsense2 as rs

class RealSenseIMUPublisher:
    def __init__(self):
        
        # Initialize ROS node
        rospy.init_node('realsense_imu_publisher', anonymous=True)

        # Fetch rates from ROS parameters, with default values if not set
        accel_fps = rospy.get_param('~accel_fps', 200)  
        gyro_fps = rospy.get_param('~gyro_fps', 200)

        # Validate accel_fps and gyro_fps
        valid_accel_fps = [100, 200]
        valid_gyro_fps = [200, 400]

        if accel_fps not in valid_accel_fps or gyro_fps not in valid_gyro_fps:
            err_msg = f"Invalid accel_fps ({accel_fps}) or gyro_fps ({gyro_fps}). Please ensure accel_fps is in {valid_accel_fps} and gyro_fps is in {valid_gyro_fps}."
            rospy.logerr(err_msg)
            raise ValueError(err_msg)

        # Print the firmware version of the connected RealSense device
        print("RealSense Firmware Version:", rs.context().devices[0].get_info(rs.camera_info.firmware_version))
        
        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Configure streams with validated parameters
        self.config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, accel_fps)
        self.config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, gyro_fps)

        # Start the pipeline
        self.pipeline.start(self.config)

        # Create a publisher for the IMU data
        self.imu_publisher = rospy.Publisher('/rs_imu', Imu, queue_size=1)

        # Set the desired publishing rate as the minimum of accel_fps and gyro_fps
        hz = min(accel_fps, gyro_fps)
        self.rate = rospy.Rate(hz)

        # Create an IMU message
        self.imu_msg = Imu()

              
    def publish_imu_data(self):
        try:
            while not rospy.is_shutdown():
                # Wait for a coherent pair of frames: IMU and RGB
                frames = self.pipeline.wait_for_frames()

                # Extract IMU data
                accel_frame = frames.first_or_default(rs.stream.accel, rs.format.motion_xyz32f)
                gyro_frame = frames.first_or_default(rs.stream.gyro, rs.format.motion_xyz32f)

                if accel_frame and gyro_frame:
                    accel_data = accel_frame.as_motion_frame().get_motion_data()
                    gyro_data = gyro_frame.as_motion_frame().get_motion_data()

                    
                    self.imu_msg.header.stamp = rospy.Time.now()

                    self.imu_msg.linear_acceleration.x = accel_data.x
                    self.imu_msg.linear_acceleration.y = accel_data.y
                    self.imu_msg.linear_acceleration.z = accel_data.z

                    self.imu_msg.angular_velocity.x = gyro_data.x
                    self.imu_msg.angular_velocity.y = gyro_data.y
                    self.imu_msg.angular_velocity.z = gyro_data.z

                    self.imu_publisher.publish(self.imu_msg)

                # Sleep to maintain the desired publishing rate
                self.rate.sleep()

        finally:
            # Stop the pipeline when the loop is interrupted
            self.pipeline.stop()

if __name__ == "__main__":
    # our second imu streams also with 50 hz
    # but up to 250 hz are possible
    imu_publisher = RealSenseIMUPublisher() 
    imu_publisher.publish_imu_data()
