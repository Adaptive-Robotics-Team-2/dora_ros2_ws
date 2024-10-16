#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Twist
from tf_transformations import quaternion_from_euler
import math
import tf2_ros
import serial

class WheelController(Node):
    def __init__(self):
        super().__init__('wheel_controller')

        # Parameters for wheel base and odometry
        self.R = 0.04  # meters (radius of the wheels)
        self.L = 0.112  # meters (length between front and rear wheels)
        self.W = 0.16 # meters (width between left and right wheels)

        # Serial communication settings
        try:
            self.serial_port = serial.Serial('/dev/ttyS0', 9600, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            raise

        # Variables to store position and orientation
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_wheel_data = [0, 0, 0, 0]  # For four wheels
        self.dt = 0.1 # seconds (time interval for the encoder ticks)
        self.TICKS_PER_REV = 1440  # Number of encoder ticks per revolution
    
        self.last_time = self.get_clock().now()

        # Publishers and Subscribers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timer for odometry updates
        self.create_timer(0.01, self.update_odometry)  # 100 Hz
            #updates more than wheel data is recieved - might cause error
            #potential fix is :if wheel_data is None:
            #return  # Skip odometry update if no data is available

        self.broadcast_static_tf()

    def read_wheel_data(self):
        """ Read encoder data from the robot via serial communication """
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                self.get_logger().info(f"Raw serial data: {line}")
                delta_ticks = [int(x) for x in line.split(',')]
                return delta_ticks
            else:
                return None
        except Exception as e:
            self.get_logger().error(f"Error reading serial data: {e}")
            return None

    def compute_holonomic_odometry(self, delta_ticks):
        """ Compute the robot's odometry based on wheel encoder data """
        def calculate_velocities(delta_ticks):
            # Convert tick to angular velocities (rad/s)
            omega = [2 * math.pi * (ticks / self.TICKS_PER_REV) / self.dt for ticks in delta_ticks]

            # Compute the linear velocities (m/s)
            vx = (self.wheel_radius / 4) * (omega[0] + omega[1] + omega[2] + omega[3])
            vy = (self.wheel_radius / 4) * (-omega[0] + omega[1] - omega[2] + omega[3])
            vz = (self.wheel_radius / (4 * (self.L + self.W))) * (-omega[0] + omega[1] + omega[2] - omega[3])

            return vx, vy, vz

        # Compute the robot's pose using the kinematic model
        vx, vy, vz = calculate_velocities(delta_ticks)

        # Compute the change in position and orientation
        self.x += (vx * self.dt * math.cos(self.th)) - (vy * self.dt * math.sin(self.th))
        self.y += (vx * self.dt * math.sin(self.th)) + (vy * self.dt * math.cos(self.th))
        self.theta += vz * self.dt

        self.get_logger().info(f'x: {self.x:.2f}, y: {self.y:.2f}, th: {self.th:.2f} (rad)')

        return self.x, self.y, self.th, vx, vy, vz

    def publish_odometry(self):
        """ Publish the odometry message and broadcast the TF """
        wheel_data = self.read_wheel_data()
        if wheel_data is None:
            return

        x, y, th, vx, vy, vth = self.compute_holonomic_odometry(wheel_data)
        current_time = self.get_clock().now()

        # Create and populate the Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom_quat = quaternion_from_euler(0, 0, th)
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)

        self.broadcast_dynamic_tf(x, y, th)

    def broadcast_dynamic_tf(self, x, y, th):
        """ Broadcast the transform for the robot's movement """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        # Set translation
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        # Convert from Euler to Quaternion
        quat = quaternion_from_euler(0, 0, th)

        # Set the quaternion orientation
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

    def cmd_vel_callback(self, msg: Twist):
        """ Handle incoming cmd_vel messages and send wheel velocities """
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        v_fl, v_fr, v_rl, v_rr = self.compute_wheel_velocities(vx, vy, wz)

        self.get_logger().info(f'FL: {v_fl:.2f}, FR: {v_fr:.2f}, RL: {v_rl:.2f}, RR: {v_rr:.2f} (rad/s)')
        self.serial_port.write(f'{v_fl:.2f},{v_fr:.2f},{v_rl:.2f},{v_rr:.2f}\n'.encode())

    def compute_wheel_velocities(self, vx, vy, wz):
        """ Compute the individual wheel velocities for a mecanum drive """
        L = self.wheel_base_length
        W = self.wheel_base_width
        r = self.wheel_radius

        v_fl = (1 / r) * (vx - vy - (L + W) * wz)
        v_fr = (1 / r) * (vx + vy + (L + W) * wz)
        v_rl = (1 / r) * (vx + vy - (L + W) * wz)
        v_rr = (1 / r) * (vx - vy + (L + W) * wz)
        
        return v_fl, v_fr, v_rl, v_rr
    
    def broadcast_static_tf(self):
        """ Broadcast a static transform between base_link and laser """
        static_transform_stamped = TransformStamped()

        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = 'base_footprint'
        static_transform_stamped.child_frame_id = 'laser'

        # Laser is 6 cm above base_link
        static_transform_stamped.transform.translation.x = 0.0
        static_transform_stamped.transform.translation.y = 0.0
        static_transform_stamped.transform.translation.z = 0.06

        # No rotation between base_link and laser
        quat = quaternion_from_euler(0, 0, math.pi)
        static_transform_stamped.transform.rotation.x = quat[0]
        static_transform_stamped.transform.rotation.y = quat[1]
        static_transform_stamped.transform.rotation.z = quat[2]
        static_transform_stamped.transform.rotation.w = quat[3]

        # Publish the static transform
        self.static_tf_broadcaster.sendTransform(static_transform_stamped)

    def update_odometry(self):
        """ Regular update of odometry information """
        self.publish_odometry()


def main(args=None):
    rclpy.init(args=args)

    node = WheelController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
