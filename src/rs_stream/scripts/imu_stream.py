import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import pyrealsense2 as rs

class RealSenseIMUPublisher:
    def __init__(self, hz):
        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)  # 62.5, 250 (Hz)
        self.config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)   # 200, 400 (Hz)

        # Start the pipeline
        self.pipeline.start(self.config)

        # Initialize ROS node
        rospy.init_node('realsense_imu_publisher', anonymous=True)

        # Create a publisher for the IMU data
        self.imu_publisher = rospy.Publisher('/rs_imu', Imu, queue_size=1)

        # Set the desired publishing rate
        self.rate = rospy.Rate(hz)

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

                    # Create and publish IMU message
                    imu_msg = Imu()
                    imu_msg.header = Header()
                    imu_msg.header.stamp = rospy.Time.now()

                    imu_msg.linear_acceleration.x = accel_data.x
                    imu_msg.linear_acceleration.y = accel_data.y
                    imu_msg.linear_acceleration.z = accel_data.z

                    imu_msg.angular_velocity.x = gyro_data.x
                    imu_msg.angular_velocity.y = gyro_data.y
                    imu_msg.angular_velocity.z = gyro_data.z

                    self.imu_publisher.publish(imu_msg)

                # Sleep to maintain the desired publishing rate
                self.rate.sleep()

        finally:
            # Stop the pipeline when the loop is interrupted
            self.pipeline.stop()

if __name__ == "__main__":
    # our second imu streams also with 50 hz
    # but up to 250 hz are possible
    imu_publisher = RealSenseIMUPublisher(hz=50) 
    imu_publisher.publish_imu_data()
