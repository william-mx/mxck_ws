#!/usr/bin/env python3

import rospy
import pyrealsense2 as rs
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo

def createCameraInfoMessage(intrinsics):

    # Populate the CameraInfo message
    camera_info_msg = CameraInfo()
    camera_info_msg.header.stamp = rospy.Time.now()
    camera_info_msg.header.frame_id = "camera_link_1"

    camera_info_msg.width = intrinsics.width
    camera_info_msg.height = intrinsics.height

    # Intrinsic matrix
    camera_info_msg.K = [intrinsics.fx, 0, intrinsics.ppx,
                            0, intrinsics.fy, intrinsics.ppy,
                            0, 0, 1]

    # Distortion parameters (assuming plumb-bob model)
    camera_info_msg.distortion_model = "plumb_bob"
    camera_info_msg.D = list(intrinsics.coeffs)

    # Rectification and projection matrices
    camera_info_msg.R = [1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0]
    camera_info_msg.P = [intrinsics.fx, 0, intrinsics.ppx, 0,
                            0, intrinsics.fy, intrinsics.ppy, 0,
                            0, 0, 1, 0]
    
    return camera_info_msg

def validate_config(width, height, fps):
    valid_sizes = [
        (1920, 1080), (1280, 720), (960, 540), 
        (848, 480), (640, 480), (640, 360), 
        (424, 240), (320, 240), (320, 180)
    ]
    size = (width, height)

    if size not in valid_sizes:
        raise ValueError(f"Unsupported image resolution {size}. Valid options are: {valid_sizes}")

    valid_fps = [6, 15, 30]  if size in [(1920, 1080), (1280, 720)] else [6, 15, 30, 60]

    if fps not in valid_fps:
        raise ValueError(f"Unsupported FPS {fps} for resolution {size}. Valid FPS options are: {valid_fps}")

def main():
    rospy.init_node('realsense_compressed_publisher', anonymous=True)
    
    # Retrieve parameters from the ROS parameter server
    width = rospy.get_param('~width', 640)
    height = rospy.get_param('~height', 360)
    fps = rospy.get_param('~fps', 30)

    # Validate configuration
    try:
        validate_config(width, height, fps)
    except ValueError as e:
        rospy.logerr(e)
        return

    # Log the publishing topic
    rospy.loginfo("Publishing compressed images from RealSense camera")

    # RealSense capture setup
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    pipeline.start(config)

    pub = rospy.Publisher('/camera/color/image_jpeg', CompressedImage, queue_size=1)

    info_pub = rospy.Publisher('/camera/color/camera_info', CameraInfo, queue_size=1, latch=True)

    published_cam_info = False

    try:
        while not rospy.is_shutdown():
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            
            # Get intrinsics of the color sensor
            intrinsics = color_frame.get_profile().as_video_stream_profile().get_intrinsics()

            # Convert to CameraInfo ROS Message
            info_msg = createCameraInfoMessage(intrinsics)

            # Publish CameraInfo once
            if not published_cam_info:
                info_pub.publish(info_msg)
                published_cam_info = True

            # Get current timestamp
            timestamp = rospy.Time.now()

            color_image = np.asanyarray(color_frame.get_data())
            
            # Encode image as JPEG
            _, img_encoded = cv2.imencode('.jpg', color_image)
            
            # Create CompressedImage message
            msg = CompressedImage()
            msg.header.stamp = timestamp
            msg.format = "jpeg"
            msg.data = np.array(img_encoded).tobytes()
            
            # Publish the compressed image
            pub.publish(msg)
                
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
