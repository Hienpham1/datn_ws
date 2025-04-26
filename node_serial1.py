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
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('topic_velocities', 'topic_velocities')
        
        # Get parameters
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.topic_velocities = self.get_parameter('topic_velocities').get_parameter_value().string_value
        
        # Initialize variable
        self.ser = None
        # Publishers and Subscribers
        self.velo_sub = self.create_subscription(Int16MultiArray, self.topic_velocities, self.velocities_callback, 10)
        # Connect to serial
        self.connect_serial()
        # Initialize motors
        for _ in range(2):
            self.write_serial(1, 0, 0)
        for _ in range(5):
            self.write_serial(0, 0, 0)

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=1)
            self.get_logger().info(f"Serial port {self.port} initialized and opened.")
        except serial.SerialException as e:
            self.get_logger().error(f"Unable to open serial port {self.port}: {e}")
            time.sleep(5)

    def write_serial(self, byte_reset, v_banh_trai, v_banh_phai):
        dir_trai = 1 if v_banh_trai < 0 else 0
        dir_phai = 1 if v_banh_phai < 0 else 0
        v_banh_trai = abs(v_banh_trai)
        v_banh_phai = abs(v_banh_phai)
        v_banh_trai = min(v_banh_trai, 255)
        v_banh_phai = min(v_banh_phai, 255)

        chieu_trai = (dir_trai << 7) | 0x01
        chieu_phai = (dir_phai << 7) | 0x02

        data = bytes([
        	byte_reset,
        	chieu_trai,
        	v_banh_trai,
        	chieu_phai,
        	v_banh_phai,
        	0x00  # padding hoặc checksum 
    	])
        self.ser.write(data)

    def velocities_callback(self, msg):
        self.write_serial(0, msg.data[0], msg.data[1])
    
def main(args=None):
    rclpy.init(args=args)
    node = ConnectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
