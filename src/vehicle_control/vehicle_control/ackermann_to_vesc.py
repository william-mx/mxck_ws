#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
import time
from rclpy.qos import qos_profile_sensor_data

class AckermannToVesc(Node):
    def __init__(self):
        super().__init__('ackermann_to_vesc')
        
        # Declare required parameters without defaults
        self.declare_parameters(
            namespace='',
            parameters=[
                ('control_type', 'rc'),
                ('servo_mid', rclpy.Parameter.Type.DOUBLE),
                ('servo_max', rclpy.Parameter.Type.DOUBLE),
                ('servo_min', rclpy.Parameter.Type.DOUBLE),
                ('erpm_min', rclpy.Parameter.Type.INTEGER),
                ('brake_amps', rclpy.Parameter.Type.DOUBLE),
                ('speed_to_erpm_gain', rclpy.Parameter.Type.INTEGER),
                ('steer_to_servo_gain', rclpy.Parameter.Type.DOUBLE),
                ('rc_dead_value', rclpy.Parameter.Type.INTEGER),
                ('rc_auto_value', rclpy.Parameter.Type.INTEGER),
                ('rc_manu_value', rclpy.Parameter.Type.INTEGER),
                ('rc_mode_button', rclpy.Parameter.Type.INTEGER),
                ('joy_dead_value', rclpy.Parameter.Type.INTEGER),
                ('joy_auto_value', rclpy.Parameter.Type.INTEGER),
                ('joy_manu_value', rclpy.Parameter.Type.INTEGER),
                ('joy_mode_button', rclpy.Parameter.Type.INTEGER),
                ('invert_steering', rclpy.Parameter.Type.BOOL)
            ]
        )

        
        
        self.load_params()
        
        # brake if current_speed == 0.0 and current_speed != previous_speed
        self.previous_speed = 0.0

        # Create a timer that calls load_params every 10 seconds
        self.timer = self.create_timer(10.0, self.load_params)

        # Initialize mode as None indicating no mode is set initially
        self.mode = None
        
        # Initialize messages for publishing speed and servo position
        self.erpm_msg = Float64()
        self.servo_msg = Float64()
        
        # Initialize brake message
        self.brake_msg = Float64()
        self.brake_msg.data = self.brake_amps
        
        # Safety check parameters
        self.speed_values = []  # Stores speed values for safety check
        n_seconds = 8  # Duration for the safety check in seconds
        hz = 40  # Expected number of speed values per second
        self.min_values = n_seconds * hz  # Minimum number of values for a valid safety check
        
        self.qos_profile = qos_profile_sensor_data
        self.qos_profile.depth = 1

        # Create subscribers
        self.safety_sub = self.create_subscription(AckermannDriveStamped, '/rc/ackermann_cmd', self.safety_check, self.qos_profile)
        
        # Driving command subscribers, initially not active
        self.rc_sub = None
        self.ad_sub = None
        
        # Joystick subscriber for mode updates
        if self.control_type == 'rc':
            self.joy_sub = self.create_subscription(Joy, '/rc/joy', self.update_mode, self.qos_profile)

        elif self.control_type == 'joy':
            self.joy_sub = self.create_subscription(Joy, '/joy', self.update_mode, self.qos_profile)
        else:
            self.get_logger().error(f'Invalid control_type: {self.control_type}')
            raise ValueError(f'control_type must be either "rc" or "joy", got {self.control_type}')
        
        # Create publishers
        self.erpm_pub = self.create_publisher(Float64, '/commands/motor/speed', 1)
        self.servo_pub = self.create_publisher(Float64, '/commands/servo/position', 1)
        self.brake_pub = self.create_publisher(Float64, '/commands/motor/brake', 1)
        
        # Inform user about safety check procedure
        self.get_logger().info(f"Please activate 'Deadman' mode. Do not touch the throttle or steering for {n_seconds} seconds. "
            "Safety check ends when speed stays at 0 m/s during this time.")
    
    def signal_calibration_complete(self):
        hz = 40
        
        amplitude = 0.2
        frequency = 3.0
        vertical_shift = 0.5
        
        t = np.linspace(0, np.pi, 60)
        values = amplitude * np.sin(frequency * t) + vertical_shift
        
        for value in values:
            msg = Float64()
            msg.data = value
            self.servo_pub.publish(msg)
            time.sleep(1/hz)
    
    def initialize_subscribers(self):
        """Initialize subscribers for manual and autonomous driving commands."""
        if self.rc_sub is None and self.ad_sub is None:
            self.rc_sub = self.create_subscription(AckermannDriveStamped, '/rc/ackermann_cmd', 
                                                   lambda x: self.callback(x, self.manu_val), self.qos_profile)
            self.ad_sub = self.create_subscription(AckermannDriveStamped,'/autonomous/ackermann_cmd',
                                                   lambda x: self.callback(x, self.auto_val),self.qos_profile)
    
    def brake(self):
        # Publishes the brake message hz times in rapid succession
        hz = 420
        
        for _ in range(hz):
            self.brake_pub.publish(self.brake_msg)
            time.sleep(1/hz)
    
    def update_mode(self, msg):
        """Update the driving mode based on joystick input."""
        new_mode = msg.buttons[self.mode_btn]

        if new_mode != self.mode:  # mode change
            self.brake()  # emergency brake
            
            self.mode = new_mode
            mode_name = {
                self.dead_val: "Deadman",
                self.auto_val: "Autonomous",
                self.manu_val: "Manual"
            }.get(self.mode, "Unknown")
            self.get_logger().info(f"Mode changed to: {mode_name}")
    
    def safety_check(self, ackermann_msg):

        """Perform safety checks before enabling driving commands."""
        if self.mode != self.dead_val:
            return
        
        speed = ackermann_msg.drive.speed
        self.speed_values.append(speed)
        if len(self.speed_values) > self.min_values:
            self.speed_values.pop(0)
            if max(self.speed_values) == 0 and min(self.speed_values) == 0:
                self.get_logger().info("Calibration complete!")
                self.destroy_subscription(self.safety_sub)
                self.signal_calibration_complete()
                self.initialize_subscribers()
    
    def callback(self, ackermann_msg, target):
        """Process received driving commands based on the current mode."""
        if self.mode != target:
            return
        
        steering_angle = ackermann_msg.drive.steering_angle
        speed = ackermann_msg.drive.speed

        if speed == 0.0 and speed != self.previous_speed:
            self.brake()

        self.previous_speed = speed

        erpm = self.speed_to_erpm_gain * speed
        servo_value = self.servo_mid + self.steering_sign * steering_angle * self.steer_to_servo_gain
        
        self.servo_msg.data = max(min(servo_value, self.servo_max), self.servo_min)
        self.erpm_msg.data = erpm
        
        self.servo_pub.publish(self.servo_msg)
        
        if abs(erpm) < self.erpm_min:
            self.brake_pub.publish(self.brake_msg)
        else:
            self.erpm_pub.publish(self.erpm_msg)
    
    def load_params(self):
        """Load all required parameters. Raises ParameterNotDeclaredException if any are missing."""
        self.servo_mid = self.get_parameter('servo_mid').value
        self.servo_max = self.get_parameter('servo_max').value
        self.servo_min = self.get_parameter('servo_min').value
        self.erpm_min = self.get_parameter('erpm_min').value
        self.brake_amps = self.get_parameter('brake_amps').value
        self.speed_to_erpm_gain = self.get_parameter('speed_to_erpm_gain').value
        self.steer_to_servo_gain = self.get_parameter('steer_to_servo_gain').value
        self.control_type = self.get_parameter('control_type').value
        
        if self.control_type == 'rc':
            self.dead_val = self.get_parameter('rc_dead_value').value
            self.auto_val = self.get_parameter('rc_auto_value').value
            self.manu_val = self.get_parameter('rc_manu_value').value
            self.mode_btn = self.get_parameter('rc_mode_button').value
        elif self.control_type == 'joy':
            self.dead_val = self.get_parameter('joy_dead_value').value
            self.auto_val = self.get_parameter('joy_auto_value').value
            self.manu_val = self.get_parameter('joy_manu_value').value
            self.mode_btn = self.get_parameter('joy_mode_button').value

        invert_steering = self.get_parameter("invert_steering").get_parameter_value().bool_value
        self.steering_sign = -1 if invert_steering else 1

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AckermannToVesc()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
    
    return 0

if __name__ == '__main__':
    main()