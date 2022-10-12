#!/usr/bin/env python

import rospy
import message_filters
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
import rosbag
import rospkg
from datetime import datetime


'''
slop: parameter that defines the delay (in seconds) with which messages can be synchronized
allow_headerless: this option specifies whether to allow storing headerless messages with current ROS time instead of timestamp
queue_size: parameter specifies how many sets of messages it should store from each input filter (by timestamp)
            while waiting for messages to arrive and complete their "set"

disable_signals=False
By default, rospy registers signal handlers so that it can exit on Ctrl-C. In some code, you may wish to disable this, including:
You're not calling init_node() from the Python Main thread. Python only allows signals to be registered from the Main thread.
'''

class SyncedRecord:

    def __init__(self):
        
        # sequence number
        self.seq = 0

        # init parameters
        self.slop = 0.05 # 50 ms
        self.queue_size = 3

        # init topics
        self.t_im = '/camera/color/image_raw'
        self.t_ac = '/ackermann_cmd'

        # save directory 
        r = rospkg.RosPack()
        base_dir = r.get_path('synced_record')
        self.bag_dir = base_dir + '/bagfiles'

        self.bag_fpath = self.bag_dir +"/synced_bag_{:%Y_%m_%d_%H_%M_%S}".format(datetime.now())

        # open bagfile in write mode
        self.bag = rosbag.Bag(self.bag_fpath, 'w')

        # Subscripe to Image and AckermannDrive
        self.im_sub = message_filters.Subscriber(self.t_im, Image)
        self.ac_sub = message_filters.Subscriber(self.t_ac, AckermannDriveStamped)

        ts = message_filters.ApproximateTimeSynchronizer([self.im_sub, self.ac_sub], self.queue_size, self.slop, allow_headerless=True)

        ts.registerCallback(self.callback)


    def callback(self, image, ackermann_cmd):

      rospy.loginfo("Recieved set of message")

      # match sequence number for later assignment
      image.header.seq = self.seq
      ackermann_cmd.header.seq = self.seq

      self.bag.write(self.t_im, image)
      self.bag.write(self.t_ac, ackermann_cmd)
  
      self.seq += 1


if __name__ == '__main__':

  # initialize node
  rospy.init_node('synced_record', anonymous=True, disable_signals = False)

  rec = SyncedRecord()

  while not rospy.is_shutdown():
    pass
  
  rec.bag.close()
  rospy.loginfo("Save as %s.", rec.bag_fpath)
  rospy.loginfo("Shutting down synced record.")




