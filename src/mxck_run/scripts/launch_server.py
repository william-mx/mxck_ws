#!/usr/bin/env python

import roslib
roslib.load_manifest('mxck_run')

import rospy
from mxck_run.srv import *

def callback(request):
    cmd = request.command
    node = request.node
    cmd = True if cmd == 'run' else False
    return launchResponse(cmd)

def launch():
    rospy.init_node('launch_service')
    service = rospy.Service('launch', launch, callback)
    rospy.spin()
    
if __name__ == '__main__':
    launch()