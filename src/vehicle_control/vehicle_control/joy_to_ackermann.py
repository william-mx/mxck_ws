import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np


def get_interp(x_vals, y_vals):     
   return lambda x: np.interp(x, x_vals, y_vals)

class JoyControl(Node):

    def __init__(self):
        super().__init__('joy_control')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('control_type', rclpy.Parameter.Type.STRING),
                ('steering_angle_max', rclpy.Parameter.Type.DOUBLE),
                ('max_backward_speed', rclpy.Parameter.Type.DOUBLE),
                ('max_forward_speed', rclpy.Parameter.Type.DOUBLE),
                ('rc_steering_axis', rclpy.Parameter.Type.INTEGER),
                ('rc_speed_axis', rclpy.Parameter.Type.INTEGER),
                ('joy_steering_axis', rclpy.Parameter.Type.INTEGER),
                ('joy_speed_axis', rclpy.Parameter.Type.INTEGER),
                ('joy_deadzone', rclpy.Parameter.Type.DOUBLE),
                ('erpm_min', rclpy.Parameter.Type.INTEGER),
                ('speed_to_erpm_gain', rclpy.Parameter.Type.INTEGER),
            ])

        # load parameters      
        self.control_type = self.get_parameter("control_type").get_parameter_value().string_value
         
        steer_max = self.get_parameter("steering_angle_max").get_parameter_value().double_value
        max_backward_speed = self.get_parameter("max_backward_speed").get_parameter_value().double_value
        max_forward_speed = self.get_parameter("max_forward_speed").get_parameter_value().double_value
        erpm_min = self.get_parameter("erpm_min").get_parameter_value().integer_value
        speed_to_erpm_gain = self.get_parameter("speed_to_erpm_gain").get_parameter_value().integer_value
        
        # print("ERPM: ", erpm_min)
        # print("speed_to_erpm_gain: ", speed_to_erpm_gain)
        joy_deadzone = self.get_parameter("joy_deadzone").get_parameter_value().double_value

        if self.control_type == 'rc':
            self.steer_ax = self.get_parameter("rc_steering_axis").get_parameter_value().integer_value
            self.speed_ax = self.get_parameter("rc_speed_axis").get_parameter_value().integer_value

        elif self.control_type == 'joy':
            self.steer_ax = self.get_parameter("joy_steering_axis").get_parameter_value().integer_value
            self.speed_ax = self.get_parameter("joy_speed_axis").get_parameter_value().integer_value

        # erpm = speed_to_erpm_gain * speed (in m/s)
        # speed = erpm / speed_to_erpm_gain
        speed_min = erpm_min / speed_to_erpm_gain

        eps = np.finfo(np.float32).eps
        
        self.steer_mapping = get_interp((-1.0, -joy_deadzone, joy_deadzone, 1.0), \
                                        (-steer_max, 0.0, 0.0, steer_max))
        
        self.speed_mapping = get_interp((-1.0, -joy_deadzone, -joy_deadzone + eps, joy_deadzone - eps, joy_deadzone, 1.0), \
                                        (max_backward_speed, -speed_min, 0.0, 0.0, speed_min, max_forward_speed))
        
        qos_profile = qos_profile_sensor_data
        qos_profile.depth = 1
            
        # define messages
        self.ackMsg = AckermannDriveStamped()
      
      	# publish ackermann messages to VESC
        self.ackermann_pub = self.create_publisher(AckermannDriveStamped, '/rc/ackermann_cmd', qos_profile)
        
        # subscribe to joy
        self.joy_sub = self.create_subscription(Joy, '/rc/joy', self.callback, qos_profile)
            
        self.joy_sub  # prevent unused variable warning             
         
    def callback(self, msg):
    
        steering_val = msg.axes[self.steer_ax]
        speed_val = msg.axes[self.speed_ax]

        steering_angle = self.steer_mapping(steering_val)
        speed = self.speed_mapping(speed_val)

        self.ackMsg.header.stamp = Clock().now().to_msg()
        self.ackMsg.drive.steering_angle = steering_angle
        self.ackMsg.drive.speed = speed

        self.ackermann_pub.publish(self.ackMsg)


def main(args=None):
    rclpy.init(args=args)

    ctrl = JoyControl()

    rclpy.spin(ctrl)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ctrl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
