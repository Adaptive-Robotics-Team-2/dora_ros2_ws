import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class WheelVelocityNode(Node):
    def __init__(self):
        super().__init__('wheel_vel_node')
        
        
        # Parameters for wheel base and odometry
        self.wheel_radius = 0.08  # meters (radius of the wheels)

        self.wheel_base_length = 0.280  # meters (length between front and rear wheels)
        self.wheel_base_width = 0.360  # meters (width between left and right wheels)
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        # Publisher for wheel velocities type String
        self.publisher_ = self.create_publisher(String, 'wheel_vel', 10)
        self.get_logger().info('Wheel Velocity Node has been started.')

    def cmd_vel_callback(self, msg: Twist):
        """ Callback function for receiving wheel velocities """
        self.get_logger().info('Result callback triggered')
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        
        wheel_velocities = self.compute_wheel_velocities(vx, vy, wz)
        
        # Publish the wheel velocities as a string
        msg = String()
        msg.data = f'{wheel_velocities[0]:.2f},{wheel_velocities[1]:.2f},{wheel_velocities[2]:.2f},{wheel_velocities[3]:.2f}'
        
        # Compute the individual wheel velocities for a mecanum drive
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        
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


def main(args=None):
    rclpy.init(args=args)
    node = WheelVelocityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
