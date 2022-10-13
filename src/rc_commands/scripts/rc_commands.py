#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16MultiArray
import rospkg
import os
import subprocess

# load parameters
mode_min_pwm = rospy.get_param("/mode_min_pwm")
mode_mid_pwm = rospy.get_param("/mode_mid_pwm")
mode_max_pwm = rospy.get_param("/mode_max_pwm")

class rc_commands:

    def __init__(self):

        # define threshold to filter out noise
        self.mode_pwm_threshold = 100

        # subscribe to pwm signals from rc receiver
        self.rc_sub = rospy.Subscriber('/veh_remote_ctrl', UInt16MultiArray, self.callback)

        # define modes
        self.previous_mode = None
        self.current_mode = None
        self.mode_changed = False
        self.is_connected = True

        # get package directory 
        r = rospkg.RosPack()
        self.pkg_dir = r.get_path('rc_commands')

    def parse_pwm(self, pwm_signal):
        steering_pwm, throttle_pwm, mode_pwm = pwm_signal

        rospy.loginfo("PWM: %s!", mode_pwm)

        # connection lost if pwm = 0
        if 0 in pwm_signal:
            self.is_connected = False
            return
        
        # mode
        if abs(mode_min_pwm - mode_pwm) < self.mode_pwm_threshold:
            mode = 3
        elif abs(mode_mid_pwm - mode_pwm) < self.mode_pwm_threshold:
            mode = 2
        elif abs(mode_max_pwm - mode_pwm) < self.mode_pwm_threshold:
            mode = 1
        else:
            self.is_connected = False
            return
        
        if mode != self.current_mode:
            self.previous_mode = self.current_mode
            self.current_mode = mode
            self.mode_changed = True

    def record_topic(self):

        if not hasattr(self, 'is_recording'):
            self.is_recording = False

        if not self.is_recording:

            rospy.loginfo("Recording starts...")

            # define topics [t1 ... tn]
            t1 = '/uss_values'

            # bagfile directory
            record_path = self.pkg_dir + '/bagfiles'

            command = 'rosbag record' + ' ' + t1

            p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True,
                executable='/bin/bash', cwd=record_path)

            self.is_recording = True
        
        else:

            rospy.loginfo("Recording stopped!")

            list_node = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
            list_output = list_node.stdout.read().decode("utf-8") 
            retcode = list_node.wait()
            assert retcode == 0, "List command returned %d" % retcode

            for str in list_output.split("\n"):
                if 'record_' in str:
                    os.system("rosnode kill " + str)
            
            self.is_recording = False


    def send_command(self):
        if self.current_mode == 1:
            self.record_topic()
        if self.current_mode == 2:
            pass
        if self.current_mode == 3:
            pass

    def callback(self,data):

        self.parse_pwm(data.data)

        if self.is_connected:
            if self.mode_changed:
                rospy.loginfo("Mode changed!")
                self.send_command()
                self.mode_changed = False


if __name__ == '__main__':

    # initialize node
    rospy.init_node('rc_commands', anonymous=True)

    rc = rc_commands()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down rc control")