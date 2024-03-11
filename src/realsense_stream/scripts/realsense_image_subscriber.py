#!/usr/bin/env python3

import rospy
import sys
import os
import rospkg

sys.path.append(os.path.join(rospkg.RosPack().get_path('realsense_stream'), 'include'))
from image_subscriber import ImageSubscriber

def process_images(host, port):
    # Instantiate the ImageSubscriber with the provided host and port
    subscriber = ImageSubscriber(host, port)

    try:
        while not rospy.is_shutdown():
            # Wait for an image frame from the subscriber
            timestamp, image = subscriber.wait_for_frame()
            ros_timestamp = rospy.Time.from_sec(timestamp)
            
            # Log receipt of the image using rospy's logging system
            rospy.loginfo(f"Received image at {ros_timestamp.to_sec()}: Shape {image.shape}")
    except rospy.ROSInterruptException:
        # Handle the ROS interrupt exception for graceful shutdown
        rospy.loginfo("ROS Interrupt received, shutting down the image processing node.")
    finally:
        # Ensure the subscriber is properly closed during shutdown
        subscriber.close()

def main():
    rospy.init_node('image_listener_node', anonymous=True)

    # Retrieve configuration parameters from the ROS parameter server
    port = rospy.get_param('~port', 5555)
    host = rospy.get_param('~host', "localhost")

    # Start processing images with the configured host and port
    process_images(host, port)

if __name__ == '__main__':
    main()
