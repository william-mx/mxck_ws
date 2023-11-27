#!/usr/bin/env python

import roslib
roslib.load_manifest('mxck_run')

import sys

from mxck_run.srv import *
import rospy
import signal
from subprocess import Popen, PIPE
import os

class LaunchServer():
    def __init__(self):
        self.PID = None
        self.process = None
        self.cmds = ['rosrun', 'roslaunch', 'kill']

        self.status = {
            'run_camera': False,
            'run_motors': False,
            'run_lidar': False,
            'run_micro': False,
            'run_pdc': False,
            'run_imu': False,
            'run_foxglove': False,
            'run_lights': False,
        }
        
        self.output = []
        
    def execute(self, mode, package, filename, args):
         
    
        cmd = [mode, package, filename, args]

        msg = "Running: %s" % " ".join(cmd)
        print(msg)

        self.process = Popen(cmd, shell=False, preexec_fn=os.setsid, stdout=PIPE, stderr=PIPE)

        self.PID = os.getpgid(self.process.pid)
        
        self.output.append("[%d]: %s" %(self.PID, msg))

        print(self.PID)
        
    def terminate(self, pid):

        print("Terminating process...")

        os.killpg(os.getpgid(pid), signal.SIGTERM)
        
        for i, s in enumerate(self.output):
            if str(pid) in s:
                self.output.pop(i)
                break
                
    
    def callback(self, request):
        cmd = request.command
        pkg = request.package
        fname = request.filename
        args = request.args
        pid = request.pid

        

        if cmd == 'rosrun' or cmd == 'roslaunch':
            self.execute(cmd, pkg, fname, args)
        elif cmd == 'kill':
            self.terminate(pid)
            
        
        return RemoteRunResponse(self.output)

    
if __name__ == "__main__":
    rospy.init_node('launch_service')
    
    ls = LaunchServer()
    service = rospy.Service('launch_node', RemoteRun, ls.callback)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")