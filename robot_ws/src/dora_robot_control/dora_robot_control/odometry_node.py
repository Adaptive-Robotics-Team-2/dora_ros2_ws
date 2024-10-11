import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Twist
from tf_transformations import quaternion_from_euler

import math
import tf2_ros
from std_msgs.msg import Float32MultiArray

class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry_node')
        # Parameters for wheel base and odometry
        self.wheel_radius = 0.08  # meters (radius of the wheels)
        self.wheel_base_length = 0.28  # meters (length between front and rear wheels)
        self.wheel_base_width = 0.362  # meters (width between left and right wheels)
        
        # Variables to store position and orientation
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_wheel_data = [0, 0, 0, 0]  # For four wheels
        self.last_time = self.get_clock().now()
        
        # Publishers and Subscribers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'processed_data',
            self.publish_odometry_callback,
            10)
        
        self.broadcast_static_tf()


    def publish_odometry_callback(self, msg: Float32MultiArray):
        encoder_data = msg.data
        self.get_logger().info(f"msg data: {encoder_data}")
        

        x, y, th, vx, vy, vth = self.compute_holonomic_odometry(encoder_data)
        current_time = self.get_clock().now()

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

    def compute_holonomic_odometry(self, wheel_data):
        current_time = self.get_clock().now()
        delta_time = (current_time - self.last_time).nanoseconds / 1e9  # seconds

        ticks_per_revolution = 1440
        distance_per_tick = (2 * math.pi * self.wheel_radius) / ticks_per_revolution

        delta_wheel = [
            (wheel_data[i] - self.last_wheel_data[i]) * distance_per_tick for i in range(4)
        ]
        self.last_wheel_data = wheel_data

        vx = (delta_wheel[0] + delta_wheel[1] + delta_wheel[2] + delta_wheel[3]) / 4.0
        vy = (-delta_wheel[0] + delta_wheel[1] + delta_wheel[2] - delta_wheel[3]) / 4.0
        vth = (-delta_wheel[0] + delta_wheel[1] - delta_wheel[2] + delta_wheel[3]) / (
            4.0 * (self.wheel_base_length + self.wheel_base_width) / 2.0
        )

        delta_x = vx * delta_time
        delta_y = vy * delta_time
        delta_th = vth * delta_time

        self.x += delta_x * math.cos(self.th) - delta_y * math.sin(self.th)
        self.y += delta_x * math.sin(self.th) + delta_y * math.cos(self.th)
        self.th += delta_th

        self.last_time = current_time

        return self.x, self.y, self.th, vx, vy, vth

    def broadcast_dynamic_tf(self, x, y, th):
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
        
    def broadcast_static_tf(self):
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

def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()