#!/usr/bin/env python

import cv2
import os
import rosbag
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped
import csv  

class ImageExport:
    def __init__(self, bag_fname):

        # read directory 
        r = rospkg.RosPack()
        base_dir = r.get_path('synced_record')
        self.bag_dir = base_dir + '/bagfiles'

        # export directory
        self.exp_dir = self.bag_dir + '/exports/' + bag_fname

        self.bag_fpath = self.bag_dir + '/' + bag_fname

        if not os.path.exists(self.bag_fpath):
            raise Exception("No such path %s", self.bag_fpath)

        # create export directory
        if not os.path.exists(self.exp_dir):
            os.makedirs(self.exp_dir)

        # open bag file
        self.bag = rosbag.Bag(self.bag_fpath, "r")
        
        self.bridge = CvBridge()

        # init topics
        self.t_im = '/camera/color/image_raw'
        self.t_ac = '/ackermann_cmd'
        self.topics = [self.t_im, self.t_ac]

        # csv output
        self.csv_header = ['fname', 'width', 'height',
        'steering_angle', 'speed']

        # open the file in the write mode
        self.f = open(self.exp_dir + '/summary.csv', 'w')

        # create the csv writer
        self.writer = csv.writer(self.f)

        # write a row to the csv file
        self.writer.writerow(self.csv_header)

        # csv output tabel
        self.tbl = {}


    def export_images(self):

        for topic, msg, t in self.bag.read_messages(topics = self.topics):
            
            seq = msg.header.seq # sequence

            # new entry
            if not seq in self.tbl.keys():
                self.tbl[seq] = dict.fromkeys(self.csv_header)

            # extract image
            if topic == self.t_im:
                id = seq

                try:
                    cv_im = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    
                except CvBridgeError as e:
                    print(e)

                out_fname = 'frame%04i.png' % msg.header.seq
                out_fpath = self.exp_dir + '/' + out_fname

                self.tbl[msg.header.seq]['fname'] = out_fname
                self.tbl[msg.header.seq]['width'] = msg.width
                self.tbl[msg.header.seq]['height'] = msg.height

                cv2.imwrite(out_fpath, cv_im)

            # export ackermann commands
            if topic == self.t_ac:
                self.tbl[id]['steering_angle'] = round(msg.drive.steering_angle, 3)
                self.tbl[id]['speed'] = round(msg.drive.speed, 2)
                
        self.bag.close()

        # write data to csv
        for d in self.tbl.values():
            values = [d['fname'], d['width'], 
            d['height'], d['steering_angle'],d['speed']]
            self.writer.writerow(values)

        # close the file
        self.f.close()

if __name__ == '__main__':

    bag_fname = 'synced_bag_2022_09_29_20_17_41'

    imex = ImageExport(bag_fname)
    imex.export_images()
