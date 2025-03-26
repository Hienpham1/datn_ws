import rclpy
from rclpy.node import Node
import serial
import time

class SerialTalker(Node):
    def __init__(self):
        super().__init__('serial_talker')
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        time.sleep(2)  # Đợi Arduino khởi động
        self.timer = self.create_timer(1.0, self.send_message)

    def send_message(self):
        message = "1\n"  # Gửi số 1 để bật đèn
        self.serial_port.write(message.encode('utf-8'))
        self.get_logger().info(f'Sent: {message.strip()}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialTalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
