import rclpy
from rclpy.node import Node
import serial

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.serial_port = serial.Serial('/dev/serial0', 9600, timeout=1)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        message = "Hello, Serial!"
        self.serial_port.write(message.encode('utf-8'))
        self.get_logger().info(f'Sent: {message}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()