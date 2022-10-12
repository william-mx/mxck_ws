#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16MultiArray
from ackermann_msgs.msg import AckermannDriveStamped



steering_min_pwm = rospy.get_param("/steering_min_pwm")
steering_mid_pwm = rospy.get_param("/steering_mid_pwm")
steering_max_pwm = rospy.get_param("/steering_max_pwm")

throttle_min_pwm = rospy.get_param("/throttle_min_pwm")
throttle_mid_pwm = rospy.get_param("/throttle_mid_pwm")
throttle_max_pwm = rospy.get_param("/throttle_max_pwm")

mode_min_pwm = rospy.get_param("/mode_min_pwm")
mode_mid_pwm = rospy.get_param("/mode_mid_pwm")
mode_max_pwm = rospy.get_param("/mode_max_pwm")

servo_min = rospy.get_param("/servo_min")
servo_mid = rospy.get_param("/servo_mid")
servo_max = rospy.get_param("/servo_max")

speed_min = rospy.get_param("/speed_min")
speed_mid = rospy.get_param("/speed_mid")
speed_max = rospy.get_param("/speed_max")

ackMsg = AckermannDriveStamped()
ackMsg.header.frame_id = "rc_control"

mode_pwm_threshold = 100
throttle_pwm_threshold = 12
steering_pwm_threshold = 5

pub = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=1000) 


def parse_pwm(pwm_signal):
    steering_pwm, throttle_pwm, mode_pwm = pwm_signal
    mode = 0
    if 0 in pwm_signal:
       return servo_mid, speed_mid, 0
    
    # steering
    if abs(steering_pwm - steering_mid_pwm) < steering_pwm_threshold:
       steering_ack = 0.0
    elif steering_pwm <= steering_mid_pwm:
       steering_ack = servo_min * (steering_pwm - steering_mid_pwm) / (steering_min_pwm - steering_mid_pwm)
    else:
       steering_ack = servo_max * (steering_pwm - steering_mid_pwm) / (steering_max_pwm - steering_mid_pwm)

       
    # throttle
    if abs(throttle_pwm - throttle_mid_pwm) < throttle_pwm_threshold:
       throttle_ack = 0.0
    elif throttle_pwm >= throttle_mid_pwm:
       throttle_ack = servo_min * (throttle_pwm - throttle_mid_pwm) / (throttle_min_pwm - throttle_mid_pwm)
    else:
       throttle_ack = servo_max * (throttle_pwm - throttle_mid_pwm) / (throttle_max_pwm - throttle_mid_pwm)
	 

    # mode
    if abs(mode_min_pwm - mode_pwm) < mode_pwm_threshold:
       mode = 3
    elif abs(mode_mid_pwm - mode_pwm) < mode_pwm_threshold:
       mode = 2
    elif abs(mode_max_pwm - mode_pwm) < mode_pwm_threshold:
       mode = 1
	
    
    return steering_ack, throttle_ack, mode

def callback(data):
    steering_ack, throttle_ack, mode = parse_pwm(data.data)

    ackMsg.header.stamp = rospy.Time.now()
    ackMsg.drive.steering_angle = steering_ack
    ackMsg.drive.speed = throttle_ack

    pub.publish(ackMsg)


def rc_control():

    rospy.init_node('rc_receiver', anonymous=True)

    rospy.Subscriber('/veh_remote_ctrl', UInt16MultiArray, callback)

    rospy.spin()  

if __name__ == '__main__':
    try:
        rc_control()
    except rospy.ROSInterruptException:
        pass





