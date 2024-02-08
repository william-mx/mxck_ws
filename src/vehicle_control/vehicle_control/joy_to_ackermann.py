import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

class JoyControl(Node):

    def __init__(self):
        super().__init__('joy_control')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('control_type', rclpy.Parameter.Type.STRING),
                ('servo_rad_min', rclpy.Parameter.Type.DOUBLE),
                ('servo_rad_mid', rclpy.Parameter.Type.DOUBLE),
                ('servo_rad_max', rclpy.Parameter.Type.DOUBLE),
                ('speed_min', rclpy.Parameter.Type.DOUBLE),
                ('speed_mid', rclpy.Parameter.Type.DOUBLE),
                ('speed_max', rclpy.Parameter.Type.DOUBLE),
                ('speed_clip', rclpy.Parameter.Type.DOUBLE),
                ('rc_speed_axis', rclpy.Parameter.Type.INTEGER),
                ('rc_steering_axis', rclpy.Parameter.Type.INTEGER),
                ('rc_deadman_button', rclpy.Parameter.Type.INTEGER),
                ('joy_speed_axis', rclpy.Parameter.Type.INTEGER),
                ('joy_steering_axis', rclpy.Parameter.Type.INTEGER),
                ('joy_deadman_button', rclpy.Parameter.Type.INTEGER),
                ('speed_deadzone', rclpy.Parameter.Type.DOUBLE),
                ('steer_deadzone', rclpy.Parameter.Type.DOUBLE),
            ])

        # load parameters      
        self.control_type = self.get_parameter("control_type").get_parameter_value().string_value
         
        self.servo_min = self.get_parameter("servo_rad_min").get_parameter_value().double_value
        self.servo_mid = self.get_parameter("servo_rad_mid").get_parameter_value().double_value
        self.servo_max = self.get_parameter("servo_rad_max").get_parameter_value().double_value

        self.speed_min = self.get_parameter("speed_min").get_parameter_value().double_value
        self.speed_mid = self.get_parameter("speed_mid").get_parameter_value().double_value
        self.speed_max = self.get_parameter("speed_max").get_parameter_value().double_value
        
        self.speed_deadzone = self.get_parameter("speed_deadzone").get_parameter_value().double_value
        self.steer_deadzone = self.get_parameter("steer_deadzone").get_parameter_value().double_value

        if self.control_type == 'rc':
            self.steer_ax = self.get_parameter("rc_steering_axis").get_parameter_value().integer_value
            self.speed_ax = self.get_parameter("rc_speed_axis").get_parameter_value().integer_value
            self.dead_btn = self.get_parameter("rc_deadman_button").get_parameter_value().integer_value
            self.steer_scale = 1

        elif self.control_type == 'joy':
            self.steer_ax = self.get_parameter("joy_steering_axis").get_parameter_value().integer_value
            self.speed_ax = self.get_parameter("joy_speed_axis").get_parameter_value().integer_value
            self.dead_btn = self.get_parameter("joy_deadman_button").get_parameter_value().integer_value
            self.steer_scale = -1 # switch directions
            
        self.qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
            
        # define messages
        self.ackMsg = AckermannDriveStamped()
      
      	# publish ackermann messages to VESC
        self.ackermann_pub = self.create_publisher(AckermannDriveStamped, '/rc/ackermann_cmd', 1)
        
        # subscribe to joy
        self.joy_sub = self.create_subscription(
            Joy,
            '/rc/joy',
            self.callback,
            qos_profile=self.qos_policy)
            
        self.joy_sub  # prevent unused variable warning


                                   
         
    def callback(self, msg):
    
        # deadman button
        if msg.buttons[self.dead_btn] == 0:

            steer_rad = self.servo_mid
            speed_mps = self.speed_mid

        else:

            # steering
            steer_val = msg.axes[self.steer_ax] * self.steer_scale

            if steer_val > self.steer_deadzone:
                steer_rad = steer_val * self.servo_max
            elif steer_val < -self.steer_deadzone:
                steer_rad = steer_val * self.servo_min * (-1)
            else:
                steer_rad = self.servo_mid

            # throttle
            speed_val = msg.axes[self.speed_ax]

            if speed_val > self.speed_deadzone:
                speed_mps = speed_val * self.speed_max
            elif speed_val < -self.speed_deadzone:
                speed_mps = speed_val * self.speed_min * (-1)
            else:
                speed_mps = self.speed_mid


        self.ackMsg.header.stamp = Clock().now().to_msg()
        self.ackMsg.drive.steering_angle = steer_rad
        self.ackMsg.drive.speed = speed_mps

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
