#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped
from dynamic_reconfigure.server import Server
from pid_control.cfg import pid_tuningConfig

class PIDcontrol():

    def __init__(self):

        # subscribe offset
        self.offset_sub = rospy.Subscriber('/offset', Float32, self.callback) # 20hz

        # publish ackermann
        self.ackermann_pub = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=1) 

        # define messages
        self.ackMsg = AckermannDriveStamped()
        
        # max steering angle [deg]
        self.max_angle = 20.0
        self.t_previous = rospy.Time.now()
        self.e_previous = 0
        self.P, self.I, self.D = 0, 0, 0
        
    def update_params(self, config, level):
        self.kp = config['kp']
        self.ki = config['ki']
        self.kd = config['kd']
        rospy.loginfo("Updated kp: %.2f, ki: %.2f, kd: %.2f" %(self.kp, self.ki, self.kd))
        return config
    
    def callback(self, data):
        error = data.data
        
        t = rospy.Time.now()
        dt = (t - self.t_previous).to_sec()
        de = self.e_previous - error
        self.P = error
        self.I = self.I + error * dt
        self.D = de / dt
        self.output = self.kp * self.P + self.ki * self.I + self.kd * self.D
        
        # clip output to +/- max_angle
        self.output = max(-self.max_angle, min(self.output, self.max_angle))
        rospy.loginfo("output: %.2f" %self.output)
        
        self.t_previous = t
        self.e_previous = error


if __name__ == "__main__":
    rospy.init_node("pid", anonymous = False)
    
    pid = PIDcontrol()

    srv = Server(pid_tuningConfig, pid.update_params)
    rospy.spin()
