#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
import math
import tf2_ros


class MecanumOdometry(Node):
    def __init__(self):
        super().__init__('mecanum_odometry')

        # Parameters (adjust these as needed)
        self.wheel_radius = 0.08  # 8 cm radius
        self.wheel_base_length = 0.153  # 15.3 cm base length
        self.wheel_base_width = 0.362  # 36.2 cm base width

        # Variables to store position and orientation
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_wheel_data = [0, 0, 0, 0]  # For four wheels

        # Time management for velocity calculation
        self.last_time = self.get_clock().now()

        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)

        # TF Broadcasters
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Timer to regularly call the update function
        self.create_timer(0.1, self.update_odometry)  # 10 Hz update rate

        # Broadcast static transform between base_link and laser
        self.broadcast_static_tf()

    def generate_constant_wheel_data(self):
        """
        Simulate constant encoder data for all four wheels.
        This is used for testing purposes.
        """
        return [100, 100, 100, 100]  # Simulated constant encoder ticks

    def compute_holonomic_odometry(self, wheel_data):
        """
        Compute the robot's odometry based on the wheel encoder data.
        Assumes mecanum wheels or omni-wheels with a known configuration.
        """

        # Update current time and compute delta time
        current_time = self.get_clock().now()
        delta_time = (current_time - self.last_time).nanoseconds / 1e9  # Convert nanoseconds to seconds

        # Parameters
        ticks_per_revolution = 1440  # Your encoder's CPR
        distance_per_tick = (2 * math.pi * self.wheel_radius) / ticks_per_revolution

        # Calculate the delta encoder values (assuming constant encoder values)
        delta_wheel = [
            (wheel_data[i] - self.last_wheel_data[i]) * distance_per_tick for i in range(4)
        ]
        self.last_wheel_data = wheel_data

        # Holonomic robot velocity calculations (for mecanum or omni-wheels)
        vx = (delta_wheel[0] + delta_wheel[1] + delta_wheel[2] + delta_wheel[3]) / 4.0
        vy = (-delta_wheel[0] + delta_wheel[1] + delta_wheel[2] - delta_wheel[3]) / 4.0
        vth = (-delta_wheel[0] + delta_wheel[1] - delta_wheel[2] + delta_wheel[3]) / (
            4.0 * (self.wheel_base_length + self.wheel_base_width) / 2.0
        )

        # Update position
        delta_x = vx * delta_time
        delta_y = vy * delta_time
        delta_th = vth * delta_time

        self.x += delta_x * math.cos(self.th) - delta_y * math.sin(self.th)
        self.y += delta_x * math.sin(self.th) + delta_y * math.cos(self.th)
        self.th += delta_th

        # Update last time
        self.last_time = current_time

        return self.x, self.y, self.th, vx, vy, vth

    def publish_odometry(self):
        # Generate constant simulated wheel data
        wheel_data = self.generate_constant_wheel_data()

        # Compute the odometry
        x, y, th, vx, vy, vth = self.compute_holonomic_odometry(wheel_data)

        # Create the Odometry message
        odom_quat = quaternion_from_euler(0, 0, th)  # Convert euler angles to quaternion
        current_time = self.get_clock().now()

        # Publish the odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'

        # Set the position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]

        # Set the velocity
        odom.child_frame_id = 'base_footprint'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        # Publish the message
        self.odom_pub.publish(odom)

        # Broadcast the dynamic transform (odom -> base_link)
        self.broadcast_dynamic_tf(x, y, th)

    def broadcast_dynamic_tf(self, x, y, th):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        quat = quaternion_from_euler(0, 0, th)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

    def broadcast_static_tf(self):
        """
        Broadcast a static transform between base_link and laser, where the laser is 6 cm above base_link.
        """
        static_transform_stamped = TransformStamped()

        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = 'base_footprint'
        static_transform_stamped.child_frame_id = 'laser'

        # Laser is 6 cm above base_link
        static_transform_stamped.transform.translation.x = 0.0
        static_transform_stamped.transform.translation.y = 0.0
        static_transform_stamped.transform.translation.z = 0.06

        # No rotation between base_link and laser
        quat = quaternion_from_euler(0, 0, 0)
        static_transform_stamped.transform.rotation.x = quat[0]
        static_transform_stamped.transform.rotation.y = quat[1]
        static_transform_stamped.transform.rotation.z = quat[2]
        static_transform_stamped.transform.rotation.w = quat[3]

        # Publish the static transform
        self.static_tf_broadcaster.sendTransform(static_transform_stamped)

    def update_odometry(self):
        self.publish_odometry()


def main(args=None):
    rclpy.init(args=args)

    mecanum_odometry = MecanumOdometry()

    try:
        rclpy.spin(mecanum_odometry)
    except KeyboardInterrupt:
        pass

    mecanum_odometry.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

