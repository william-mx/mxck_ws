

# ROS Package for Efficient RealSense Image Streaming

This ROS package serves as a practical alternative to the official `realsense_camera` package, offering significant performance improvements on constrained systems. It comprises two main scripts:

### `realsense_image_publisher.py` (Publisher)
- **Functionality:** Captures images from a RealSense camera and publishes them over a network using ZeroMQ. 
- **Configuration:** Allows for customizable settings, including image resolution (FPS), publishing port, and more, to optimize performance according to system capabilities.

### `realsense_image_subscriber.py` (Subscriber)
- **Functionality:** A sample node that subscribes to the image stream published by `realsense_image_publisher.py`. It demonstrates how to receive and process images efficiently.
- **Integration:** Utilizes the `ImageSubscriber` module from `include/image_subscriber.py` for handling image reception and processing in a separate thread, ensuring minimal impact on the main program's performance.

## Key Advantages
- **Reduced CPU Load:** Specially designed to minimize computational demand, making it ideal for hardware with limited processing capacity.
- **Flexible Configuration:** The publisher script's settings can be easily adjusted, offering the flexibility to balance performance and resource utilization.
- **Streamlined Workflow:** Provides a straightforward approach for real-time image streaming and processing with ROS and ZeroMQ.

