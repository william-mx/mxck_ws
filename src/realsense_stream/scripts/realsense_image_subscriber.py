#!/usr/bin/env python3
import rospkg
import rospy
import sys
import os

# Ensure the path to the 'image_subscriber' module is correct
r = rospkg.RosPack()
sys.path.append(os.path.join(r.get_path('realsense_stream'), 'include'))
print(sys.version)

from image_subscriber import ImageSubscriber

def callback(image):
    print(image.shape)

def main():
    # Initialize the ROS node
    rospy.init_node('image_listener_node', anonymous=True)

    # Retrieve parameters from the ROS parameter server
    port = rospy.get_param('~port', 5555)
    host = rospy.get_param('~host', "localhost")
    
    # Instantiate the ImageSubscriber with the defined callback
    im_sub = ImageSubscriber(callback, host, port)

    # Keep the program alive.
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
