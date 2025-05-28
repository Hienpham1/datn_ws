import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import serial
import time
import math


class ConnectNode(Node):
    def __init__(self):
        super().__init__('connect_node')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('topic_velocities', 'topic_velocities')
        self.declare_parameter('topic_paint', 'topic_paint')
        
        # Get parameters
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.topic_velocities = self.get_parameter('topic_velocities').get_parameter_value().string_value
        self.topic_paint = self.get_parameter('topic_paint').get_parameter_value().string_value
        
        # Initialize variable
        self.ser = None
        self.paint_state = 0
        # Publishers and Subscribers
        self.velo_sub = self.create_subscription(Int16MultiArray, self.topic_velocities, self.velocities_callback, 10)
        self.paint_sub = self.create_subscription(Int16MultiArray, self.topic_paint, self.paint_callback, 10)
        # Connect to serial
        self.connect_serial()
        # Initialize motors on paint
        for _ in range(2):
            self.write_serial(1, 0, 0, 0)
        for _ in range(5):
            self.write_serial(0, 0, 0, 0)

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=1)
            self.get_logger().info(f"Serial port {self.port} initialized and opened.")
        except serial.SerialException as e:
            self.get_logger().error(f"Unable to open serial port {self.port}: {e}")
            time.sleep(5)

    def write_serial(self, byte_reset, pwm_left, pwm_right, paint_state):
        dir_left = 1 if pwm_left < 0 else 0
        dir_right = 1 if pwm_right < 0 else 0
        pwm_left = abs(pwm_left)
        pwm_right = abs(pwm_right)
        pwm_left = min(pwm_left, 255)
        pwm_right = min(pwm_right, 255)

        dir_left = (dir_left << 7) | 0x01
        dir_right = (dir_right << 7) | 0x02

        data = bytes([
        	byte_reset,
        	dir_left,
        	pwm_left,
        	dir_right,
        	pwm_right,
            paint_state
    	])
        self.ser.write(data)

    def paint_callback(self, msg):
        self.paint_state = 1 if msg.data else 0
        self.get_logger().info(f"Updated paint state: {self.paint_state}")
    def velocities_callback(self, msg):
        self.write_serial(0, msg.data[0], msg.data[1], self.paint_state)
    
def main(args=None):
    rclpy.init(args=args)
    node = ConnectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
