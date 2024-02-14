import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.logging import get_logger

from mxcarkit_vehctrl_message.msg import VehCtrlCustomMessage
from sensor_msgs.msg import Joy


class RCJoystick(Node):

    def __init__(self):
        super().__init__('rc_to_joy')


        self.logger = get_logger('rc_to_joy')

        self.qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('steering_min_pwm', rclpy.Parameter.Type.INTEGER),
                ('steering_mid_pwm', rclpy.Parameter.Type.INTEGER),
                ('steering_max_pwm', rclpy.Parameter.Type.INTEGER),
                ('throttle_min_pwm', rclpy.Parameter.Type.INTEGER),
                ('throttle_mid_pwm', rclpy.Parameter.Type.INTEGER),
                ('throttle_max_pwm', rclpy.Parameter.Type.INTEGER),
                ('mode_min_pwm', rclpy.Parameter.Type.INTEGER),
                ('mode_mid_pwm', rclpy.Parameter.Type.INTEGER),
                ('mode_max_pwm', rclpy.Parameter.Type.INTEGER),
            ])
        
        # load parameters       
        self.steer_min_pwm = self.get_parameter("steering_min_pwm").get_parameter_value().integer_value
        self.steer_mid_pwm = self.get_parameter("steering_mid_pwm").get_parameter_value().integer_value
        self.steer_max_pwm = self.get_parameter("steering_max_pwm").get_parameter_value().integer_value

        self.throt_min_pwm = self.get_parameter("throttle_min_pwm").get_parameter_value().integer_value
        self.throt_mid_pwm = self.get_parameter("throttle_mid_pwm").get_parameter_value().integer_value
        self.throt_max_pwm = self.get_parameter("throttle_max_pwm").get_parameter_value().integer_value

        self.mode_min_pwm = self.get_parameter("mode_min_pwm").get_parameter_value().integer_value
        self.mode_mid_pwm = self.get_parameter("mode_mid_pwm").get_parameter_value().integer_value
        self.mode_max_pwm = self.get_parameter("mode_max_pwm").get_parameter_value().integer_value
      
        # define thresholds to filter out noise
        self.mode_pwm_threshold = 100
        self.throt_pwm_threshold = 12
        self.steer_pwm_threshold = 5
        
        # define messages
        self.joy_msg = Joy()
        self.joy_msg.header.frame_id = 'rc_control'
      
      	# publish joy message
        self.joy_pub = self.create_publisher(Joy, '/rc/joy', 2)
        
        # subscribe to pwm signals from rc receiver
        self.rc_sub = self.create_subscription(
            VehCtrlCustomMessage,
            '/veh_remote_ctrl',
            self.callback,
            qos_profile=self.qos_policy)
            


    def parse_pwm(self, pwm_signal):

        mode_pwm = pwm_signal.remote_pwm
        steer_pwm = pwm_signal.steering_pwm
        throt_pwm = pwm_signal.throttle_pwm
        
        steer_pwm, mode_pwm = mode_pwm, steer_pwm # SWITCHED!!! 
        
        steer_val, throt_val, mode_val = 0.0, 0.0, 0 # init
	
        # connection lost if pwm = 0
        if 0 in [steer_pwm, throt_pwm, mode_pwm]:
           return steer_val, throt_val, mode_val
        
        # steering
        if abs(steer_pwm - self.steer_mid_pwm) < self.steer_pwm_threshold:
           steer_val = 0.0
        elif steer_pwm >= self.steer_mid_pwm:
           steer_val = min(float(steer_pwm - self.steer_mid_pwm) / (self.steer_max_pwm - self.steer_mid_pwm), 1.0) # [0.0, 1.0]
        else:
           steer_val = max(float(steer_pwm - self.steer_mid_pwm) / (self.steer_mid_pwm - self.steer_min_pwm), -1.0) # [-1.0, 0.0]

        # throttle
        if abs(throt_pwm - self.throt_mid_pwm) < self.throt_pwm_threshold:
           throt_val = 0.0
        elif throt_pwm >= self.throt_mid_pwm:
           throt_val = min(float(throt_pwm - self.throt_mid_pwm) / (self.throt_max_pwm - self.throt_mid_pwm), 1.0) # [0.0, 1.0]
        else:
           throt_val = max(float(throt_pwm - self.throt_mid_pwm) / (self.throt_mid_pwm - self.throt_min_pwm), -1.0) # [-1.0, 0.0]
      
        # mode
        if abs(self.mode_min_pwm - mode_pwm) < self.mode_pwm_threshold:
           mode_val = 0
        elif abs(self.mode_mid_pwm - mode_pwm) < self.mode_pwm_threshold:
           mode_val =  1
        elif abs(self.mode_max_pwm - mode_pwm) < self.mode_pwm_threshold:
           mode_val =  2
        
        return steer_val, throt_val, mode_val

         
    def callback(self, msg):
        steer_val, throt_val, mode_val = self.parse_pwm(msg)
        
        self.joy_msg.axes = [steer_val, throt_val]
        self.joy_msg.buttons = [mode_val]
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
