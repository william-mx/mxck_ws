#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import sensor_msgs_py.point_cloud2 as pc2
from mxck_run.message_utils import get_relative_transform, create_point_cloud_message
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int16MultiArray
from rclpy.qos import qos_profile_sensor_data
import tf2_geometry_msgs

class UltrasonicSensorProcessor(Node):
    def __init__(self):
        super().__init__('ultrasonic_pointcloud')

        self.max_distance = 1.5  # meters
        self.min_distance = 0.02  # meters
        self.fov = 60  # degrees field of view
        self.num_points = 20  # points per wave
        self.num_waves = 20  # waves per sensor
        self.base = "base_link"  # Target frame for transformations

        self.intervals = np.linspace(self.min_distance, self.max_distance, self.num_waves)
        self.angle_start = np.deg2rad(-self.fov / 2)  # Start angle in radians
        self.angle_end = np.deg2rad(self.fov / 2)  # End angle in radians
        self.thetas = np.linspace(self.angle_start, self.angle_end, self.num_points)

        # Mapping sensor indices to tf frames
        self.index2frame = {
            0: "USS_SRB",
            1: "USS_SRF",
            2: "USS_FR",
            3: "USS_FC",
            4: "USS_FL",
            5: "USS_SLF",
            6: "USS_SLB",
            7: "USS_BL",
            8: "USS_BC",
            9: "USS_BR"
        }
        self.frames = list(self.index2frame.values())

        # Fetch transformations using the function
        self.transformations = {frame: get_relative_transform(frame, self.base) for frame in self.frames}

        # Create a QoS profile
        qos_profile = qos_profile_sensor_data
        qos_profile.depth = 1

        # Single Publisher for PointCloud2 message
        self.publisher = self.create_publisher(PointCloud2, "/pdc", qos_profile)

        # Subscriber for ultrasonic distances
        self.create_subscription(Int16MultiArray, '/uss_sensors', self.callback, qos_profile)

    def callback(self, data):
        all_points = []
        distances = data.data

        for i, distance in enumerate(distances):
            if distance <= 0:
                continue

            distance_meters = float(distance) / 100  # Convert cm to meters
            points = self.generate_points_at_waves(distance_meters)
            transformed_points = self.transform_points(points, self.index2frame[i])
            all_points.extend(transformed_points)


        pointcloud_msg = create_point_cloud_message(all_points, self.base)
        self.publisher.publish(pointcloud_msg)
        
    def generate_points_at_waves(self, measurement):
        distances = self.intervals[self.intervals < measurement]

        if len(distances) > 0:
            distances[-1] = measurement
        else:
            distances = [measurement]

        all_points = []
        for d in distances:
            x = d * np.cos(self.thetas)
            y = d * np.sin(self.thetas)
            z = np.full(self.num_points, d)
            points = np.column_stack((x, y, z))
            all_points.append(points)

        return np.vstack(all_points)

    def transform_points(self, points, from_frame):
        transformed_points = []
        for (x, y, z) in points:
            point_stamped = PointStamped()
            point_stamped.header.frame_id = from_frame
            point_stamped.header.stamp = rclpy.clock.Clock().now().to_msg()
            point_stamped.point.x = x
            point_stamped.point.y = y
            point_stamped.point.z = z
            
            transformation = self.transformations.get(from_frame)
            if transformation is None:
                self.get_logger().error(f"No transformation available for frame {from_frame}")
                continue  # Skip this point or handle it according to your application's needs

            transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transformation)
            transformed_points.append((transformed_point.point.x, transformed_point.point.y, transformed_point.point.z))

        return transformed_points


def main():
    rclpy.init()
    processor = UltrasonicSensorProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


