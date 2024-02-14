import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from math import atan

class AckermannToVesc(Node):
    def __init__(self):
        super().__init__('ackermann_to_vesc')
        
        self.load_params()

        self.mode = self.dead_val

        self.init_mapping_function()

        self.erpm_msg = Float64()
        self.servo_msg = Float64()

        self.rc_sub = self.create_subscription(AckermannDriveStamped, '/rc/ackermann_cmd', lambda x: self.callback(x, self.manu_val), 10)
        self.ad_sub = self.create_subscription(AckermannDriveStamped, '/autonomous/ackermann_cmd', lambda x: self.callback(x, self.auto_val), 10)
        self.joy_sub = self.create_subscription(Joy, '/rc/joy', self.deadman_callback, 10)

        self.erpm_pub = self.create_publisher(Float64, '/commands/motor/speed', 10)
        self.servo_pub = self.create_publisher(Float64, '/commands/servo/position', 10)


    def load_params(self):
        
        # Declare parameters
        self.declare_parameter('servo_max', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('servo_min', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('servo_mid', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('wheelbase', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('lr_rmin', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('rr_rmin', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('speed_to_erpm_gain', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('rc_deadman_button', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('rc_dead_value', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('rc_auto_value', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('rc_manu_value', rclpy.Parameter.Type.INTEGER)
        
        # VESC servo configuration
        self.servo_max = self.get_parameter('servo_max').get_parameter_value().double_value
        self.servo_min = self.get_parameter('servo_min').get_parameter_value().double_value
        self.servo_mid = self.get_parameter('servo_mid').get_parameter_value().double_value
        
        # Wheelbase
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        
        # Radius
        self.lr_rmin = self.get_parameter('lr_rmin').get_parameter_value().double_value
        self.rr_rmin = self.get_parameter('rr_rmin').get_parameter_value().double_value
        
        # Speed to erpm gain
        self.speed_to_erpm_gain = self.get_parameter('speed_to_erpm_gain').get_parameter_value().integer_value
        
        
        # Control configurations
        self.dead_btn = self.get_parameter('rc_deadman_button').get_parameter_value().integer_value
        self.dead_val = self.get_parameter('rc_dead_value').get_parameter_value().integer_value
        self.auto_val = self.get_parameter('rc_auto_value').get_parameter_value().integer_value
        self.manu_val = self.get_parameter('rc_manu_value').get_parameter_value().integer_value


    def callback(self, msg, target):

        if self.mode != target:
            return

        steer_rad = msg.drive.steering_angle
        speed = msg.drive.speed

        erpm = self.speed_to_erpm_gain * speed

        if steer_rad > 0:
            val = self.rr_m * steer_rad + self.servo_mid
        else:
            val = self.lr_m * steer_rad + self.servo_mid

        self.servo_msg.data = max(min(val, self.servo_max), self.servo_min)
        self.erpm_msg.data = erpm
        
        self.servo_pub.publish(self.servo_msg)
        self.erpm_pub.publish(self.erpm_msg)

    def deadman_callback(self, msg):
        self.mode = msg.buttons[self.dead_btn]

    def init_mapping_function(self):
        self.lr_rad_max = atan(self.wheelbase / self.lr_rmin)
        self.rr_rad_max = atan(self.wheelbase / self.rr_rmin)
        self.rr_dy = self.servo_max - self.servo_mid
        self.lr_dy = self.servo_mid - self.servo_min
        self.rr_m = self.rr_dy / self.rr_rad_max
        self.lr_m = self.lr_dy / self.lr_rad_max
        self.rad_max = max(abs(self.rr_rad_max), abs(self.lr_rad_max))

        self.get_logger().info("The maximum steering angle is set to +/- %.3f rad" % self.rad_max)

def main(args=None):
    rclpy.init(args=args)

    node = AckermannToVesc()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Ackermann to VESC Node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
