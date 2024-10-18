import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class VelocityController(Node):

        
        
    def __init__(self):
        super().__init__('velocity_controller')
        self.serial_port = serial.Serial('/dev/ttyS0', 9600, timeout=1)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):
        self.serial_port.write(b'Hello, Robot!\n')
        self.get_logger().info('Sending hello command')
                
             
            

def main(args=None):
    rclpy.init(args=args)
    velocity_controller = VelocityController()
    rclpy.spin(velocity_controller)
    velocity_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()