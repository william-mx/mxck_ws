# Efficient RealSense Image Streaming ROS Package

This ROS package provides an efficient solution for streaming images from RealSense cameras. It consists of two main scripts:

## `realsense_jpeg_publisher.py` (Publisher)
- **Functionality:** Captures images from a RealSense camera and publishes them as ROS compressed image messages with JPEG encoding.
  - Allows customization of settings such as image resolution and FPS.

## `realsense_jpeg_subscriber.py` (Subscriber)
- **Functionality:** Demonstrates a sample node that subscribes to the ROS compressed image messages published by `realsense_jpeg_publisher.py`. It also measures the FPS (Frames Per Second) of the image stream.

### Performance
- Achieves approximately 60 FPS with a resolution of (640, 360) on Jetson NX.