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

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=1)
            # Reset motor and servo+pump
            for i in range(2):
                self.ser.write(bytes([0x0B, 0, 0, 0, 0, 0, 0xFF])) # 7 byte
                self.ser.write(bytes([0x0C, 0, 0, 0, 0xFF]))       # 5 byte
            self.get_logger().info(f"Serial port {self.port} initialized and opened.")
        except serial.SerialException as e:
            self.get_logger().error(f"Unable to open serial port {self.port}: {e}")
            self.ser = None
            time.sleep(5)

    def velocities_callback(self, msg):
        # Gửi gói tốc độ (7 byte): [0x0B, dir_left, pwm_left, dir_right, pwm_right, checksum, 0xFF]
        pwm_left = max(-255, min(255, msg.data[0]))
        pwm_right = max(-255, min(255, msg.data[1]))
        
        dir_left = 1 if pwm_left < 0 else 0
        dir_right = 1 if pwm_right < 0 else 0
        pwm_left = abs(pwm_left)
        pwm_right = abs(pwm_right)
        CRC = (dir_left + dir_right + pwm_left + pwm_right) % 256
        speed_data = bytes([0x0B, dir_left, pwm_left, dir_right, pwm_right, CRC, 0xFF])
        self.ser.write(speed_data)
    
    def paint_callback(self, msg):
        # Gửi gói paint (5 byte): [0x0B, value_pump, value_servo, 0xFF]
        pump_value = max(0, min(255, int(msg.data[0])))
        servo_value = max(30, min(50, int(msg.data[1])))
        CRC = (pump_value + servo_value) % 256
        paint_data = bytes([0x0C, pump_value, servo_value, CRC, 0xFF])
        self.ser.write(paint_data)
    
def main(args=None):
    rclpy.init(args=args)
    node = ConnectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
