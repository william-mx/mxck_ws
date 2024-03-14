#!/usr/bin/env python

import rospy
import sys
import os
import rospkg
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np

sys.path.append(os.path.join(rospkg.RosPack().get_path('realsense_stream'), 'include'))
from image_subscriber import ImageSubscriber

def process_images(host, port):
    # Instantiate the ImageSubscriber with the provided host and port
    subscriber = ImageSubscriber(host, port)

    # Initialize ROS node
    rospy.init_node('image_publisher_node', anonymous=True)
    
    # Initialize CvBridge
    bridge = CvBridge()

    try:
        # Initialize ROS publisher with CompressedImage message type
        image_pub = rospy.Publisher('/camera/color/image_jpeg', CompressedImage, queue_size=1)

        while not rospy.is_shutdown():
            # Wait for an image frame from the subscriber
            timestamp, image = subscriber.wait_for_frame()
            ros_timestamp = rospy.Time.from_sec(timestamp)

            # Convert the image to JPEG compressed ROS Image message
            image_msg = bridge.cv2_to_compressed_imgmsg(image, dst_format="jpg")

            # Set timestamp
            image_msg.header.stamp = ros_timestamp

            # Publish the compressed image
            image_pub.publish(image_msg)

    except rospy.ROSInterruptException:
        # Handle the ROS interrupt exception for graceful shutdown
        rospy.loginfo("ROS Interrupt received, shutting down the image processing node.")
    finally:
        # Ensure the subscriber is properly closed during shutdown
        subscriber.close()

def main():
    # Retrieve configuration parameters from the ROS parameter server
    port = rospy.get_param('~port', 5555)
    host = rospy.get_param('~host', "localhost")

    # Start processing images with the configured host and port
    process_images(host, port)

if __name__ == '__main__':
    main()
