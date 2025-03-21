import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.logging import get_logger
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray

import numpy as np

def get_interp(x_vals, y_vals):
   return lambda x: np.interp(x, x_vals, y_vals)
   
class RCJoystick(Node):

    def __init__(self):
        super().__init__('rc_to_joy')


        qos_profile = qos_profile_sensor_data
        qos_profile.depth = 1

        self.declare_parameters(
            namespace='',
            parameters=[
                ('steering_min_pwm', rclpy.Parameter.Type.INTEGER),
                ('steering_mid_pwm', rclpy.Parameter.Type.INTEGER),
                ('steering_max_pwm', rclpy.Parameter.Type.INTEGER),
                ('speed_min_pwm', rclpy.Parameter.Type.INTEGER),
                ('speed_mid_pwm', rclpy.Parameter.Type.INTEGER),
                ('speed_max_pwm', rclpy.Parameter.Type.INTEGER),
                ('mode_min_pwm', rclpy.Parameter.Type.INTEGER),
                ('mode_mid_pwm', rclpy.Parameter.Type.INTEGER),
                ('mode_max_pwm', rclpy.Parameter.Type.INTEGER),
                ('rc_steering_axis', rclpy.Parameter.Type.INTEGER),
                ('rc_speed_axis', rclpy.Parameter.Type.INTEGER),
                ('rc_mode_button', rclpy.Parameter.Type.INTEGER),
            ])

        
        # load parameters
        self.steer_ax = self.get_parameter("rc_steering_axis").get_parameter_value().integer_value
        self.speed_ax = self.get_parameter("rc_speed_axis").get_parameter_value().integer_value
        self.mode_btn = self.get_parameter("rc_mode_button").get_parameter_value().integer_value
           
        steer_min_pwm = self.get_parameter("steering_min_pwm").get_parameter_value().integer_value
        steer_mid_pwm = self.get_parameter("steering_mid_pwm").get_parameter_value().integer_value
        steer_max_pwm = self.get_parameter("steering_max_pwm").get_parameter_value().integer_value

        speed_min_pwm = self.get_parameter("speed_min_pwm").get_parameter_value().integer_value
        speed_mid_pwm = self.get_parameter("speed_mid_pwm").get_parameter_value().integer_value
        speed_max_pwm = self.get_parameter("speed_max_pwm").get_parameter_value().integer_value

        mode_min_pwm = self.get_parameter("mode_min_pwm").get_parameter_value().integer_value
        mode_mid_pwm = self.get_parameter("mode_mid_pwm").get_parameter_value().integer_value
        mode_max_pwm = self.get_parameter("mode_max_pwm").get_parameter_value().integer_value
        
        self.mode_values = np.array([mode_min_pwm, mode_mid_pwm, mode_max_pwm])

        self.steer_mapping = get_interp((steer_min_pwm, steer_mid_pwm, steer_max_pwm), (-1.0, 0.0, 1.0))
        self.speed_mapping = get_interp((speed_min_pwm, speed_mid_pwm, speed_max_pwm), (-1.0, 0.0, 1.0))

        # define messages
        self.joy_msg = Joy()
        self.joy_msg.header.frame_id = 'rc_control'
        self.joy_msg.axes = [0.0] * (max(self.steer_ax, self.speed_ax) + 1)
        self.joy_msg.buttons = [0] * (self.mode_btn + 1)

      	# publish joy message
        self.joy_pub = self.create_publisher(Joy, '/rc/joy', qos_profile)
        
        # subscribe to pwm signals from rc receiver
        self.rc_sub = self.create_subscription(Int16MultiArray, '/veh_remote_ctrl', self.callback, qos_profile)
            


    def parse_pwm(self, pwm_signal):
        steer_pwm, speed_pwm, mode_pwm = pwm_signal[:3]
             
        steer_val, throt_val, mode_val = 0.0, 0.0, 0 # init
	
        # connection lost if pwm = 0
        if 0 in [steer_pwm, speed_pwm, mode_pwm]:
           return steer_val, throt_val, mode_val
        
        steer_val = self.steer_mapping(steer_pwm)
        speed_val = self.speed_mapping(speed_pwm)
        mode_val = np.argmin(np.abs(self.mode_values - mode_pwm))
        
        return steer_val, speed_val, mode_val

         
    def callback(self, msg):
        steer_val, throt_val, mode_val = self.parse_pwm(msg.data)
        
        self.joy_msg.axes[self.steer_ax] = steer_val
        self.joy_msg.axes[self.speed_ax] = throt_val
        
        self.joy_msg.buttons[self.mode_btn] = mode_val
    
        self.joy_msg.header.stamp = Clock().now().to_msg()
        self.joy_pub.publish(self.joy_msg)

        



def main(args=None):
    rclpy.init(args=args)

    rcjoy = RCJoystick()

    rclpy.spin(rcjoy)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rcjoy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
