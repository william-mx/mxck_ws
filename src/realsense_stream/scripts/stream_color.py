#!/usr/bin/env python3

import rospy
import pyrealsense2 as rs
import numpy as np
import cv2
import zmq

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
    rospy.init_node('realsense_zmq_publisher', anonymous=True)
    
    # Retrieve parameters from the ROS parameter server
    width = rospy.get_param('~width', 1920)
    height = rospy.get_param('~height', 1080)
    fps = rospy.get_param('~fps', 30)

    # Validate configuration
    try:
        validate_config(width, height, fps)
    except ValueError as e:
        rospy.logerr(e)
        return

    # Setup ZeroMQ publisher
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5555")

    # RealSense capture setup
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    pipeline.start(config)

    try:
        while not rospy.is_shutdown():
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            
            # Encode image as JPEG
            success, img_encoded = cv2.imencode('.jpg', color_image)
            
            # Send image
            if success:
                socket.send(img_encoded.tobytes())
                
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()
        socket.close()
        context.term()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


    