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
        self.ackermann_pub = rospy.Publisher('/autonomous/ackermann_cmd', AckermannDriveStamped, queue_size=1) 

        # define messages
        self.ackMsg = AckermannDriveStamped()
        
        # max steering angle [deg]
        self.max_angle = 0.3
        self.t_previous = rospy.Time.now()
        self.e_previous = 0
        self.P, self.I, self.D = 0, 0, 0
        
        self.speed_mps, self.max_angle = 0, 0
        
    def update_params(self, config, level):
        self.kp = config['kp']
        self.ki = config['ki']
        self.kd = config['kd']
        self.speed_mps = config['speed']
        self.max_angle = config['max_angle']
        
        rospy.loginfo("Updated kp: %.2f, ki: %.2f, kd: %.2f, speed: %.2f m/s, max_angle: %.2f rad" \
            %(self.kp, self.ki, self.kd, self.speed_mps, self.max_angle))
        
        return config
    
    def callback(self, data):
        error = data.data
        
        t = rospy.Time.now()
        dt = (t - self.t_previous).to_sec()
        de = self.e_previous - error
        self.P = error
        self.I = self.I + error * dt
        self.D = de / dt
        steer_rad = self.kp * self.P + self.ki * self.I + self.kd * self.D
        
        # clip output to +/- max_angle
        steer_rad = max(-self.max_angle, min(steer_rad, self.max_angle))
        rospy.loginfo("steering angle [rad]: %.2f, offset px: %.2f" %(steer_rad, error))
        
        self.t_previous = t
        self.e_previous = error

        self.ackMsg.header.stamp = rospy.Time.now()
        self.ackMsg.drive.steering_angle = steer_rad
        self.ackMsg.drive.speed = self.speed_mps

        self.ackermann_pub.publish(self.ackMsg)

if __name__ == "__main__":
    rospy.init_node("pid", anonymous = False)
    
    pid = PIDcontrol()

    srv = Server(pid_tuningConfig, pid.update_params)
    rospy.spin()
