#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image


class your_node_name:

  def __init__(self):

    self.topic_pub = rospy.Publisher("pub_topic_name",Image)

    self.topic_sub = rospy.Subscriber("sub_topic name",Image,self.callback)

  def callback(self,data):
    try:
      self.topic_pub.publish()
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)