import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

import serial
import time
import math


class ConnectNode(Node):
    def __init__(self):
        super().__init__('connect_node_pwm')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('topic_odom', 'odom_encoder')
        self.declare_parameter('topic_imu', 'IMU')
        self.declare_parameter('fixed_frame_odom', 'odom')
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheels_distance', 0.32)
        self.declare_parameter('pulses_per_revolution', 1000.0) # che do doc rising 
        self.declare_parameter('topic_velocities', 'topic_velocities')

        # Get parameters
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.topic_odom = self.get_parameter('topic_odom').get_parameter_value().string_value
        self.topic_imu = self.get_parameter('topic_imu').get_parameter_value().string_value
        self.fixed_frame_odom = self.get_parameter('fixed_frame_odom').get_parameter_value().string_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheels_distance = self.get_parameter('wheels_distance').get_parameter_value().double_value
        self.pulses_per_revolution = self.get_parameter('pulses_per_revolution').get_parameter_value().double_value
        self.topic_velocities = self.get_parameter('topic_velocities').get_parameter_value().string_value

        # Initialize variables
        self.ser = None
        self.rec_data = 0
        self.buff_data = [0] * 8
        self.count = 0
        self.angle = 0
        self.en1 = 0
        self.en2 = 0
        self.pre_angle = 0
        self.theta = 0
        self.x = 0
        self.y = 0
        self.pre_theta = 0
        self.pre_x = 0
        self.pre_y = 0
        self.scaling_factor = (2.0 * math.pi * self.wheel_radius) / self.pulses_per_revolution
        self.rad_to_deg = 180 / math.pi
        self.rad = math.pi / 1800
        self._check = False
        self.pre_measurement_time = time.time()
        self.pre_measurement_time_imu = time.time()

        # Publishers and Subscribers
        self.odom_pub = self.create_publisher(Odometry, self.topic_odom, 10)
        self.imu_pub = self.create_publisher(Imu, self.topic_imu, 10)
        self.create_subscription(Int16MultiArray, self.topic_velocities, self.velocities_callback, 10)

        # Connect to serial
        self.connect_serial()

        # Initialize motors
        for _ in range(2):
            self.write_serial(1, 0, 0)
        for _ in range(5):
            self.write_serial(0, 0, 0)

        # Timer for reading serial data
        self.create_timer(0.01, self.read_serial)

    def create_quaternion_msg_from_yaw(self, yaw):
        qz = math.sin(yaw * 0.5);
        qw = math.sin(yaw * 0.5)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)

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
        	0x00  # padding hoặc checksum tùy bạn
    	])
        self.ser.write(data)

    def to_signed_16(self, val):
        return val - 0x10000 if val > 0x7FFF else val

    def read_serial(self):
        while self.ser.in_waiting > 0:
            self.rec_data = self.ser.read(1)[0]
            #print(f"[serial Rx] Byte: {self.rec_data:#04x}, count: {self.count}")
            if self.count == 0 and self.rec_data != 0x02:
                break
            if self.count == 7 and self.rec_data != 0x03:
                break
            if self.count == 7 and self.rec_data == 0x03:
                self.angle = (self.buff_data[1] << 8) | self.buff_data[2]
                raw_en1 = (self.buff_data[3] << 8) | self.buff_data[4]
                raw_en2 = (self.buff_data[5] << 8) | self.buff_data[6]
                self.en1 = self.to_signed_16(raw_en1)
                self.en2 = self.to_signed_16(raw_en2)
                self.count = 0
                if not self._check:
                    if self.angle != 0:
                        break
                    else:
                        self._check = True
                self.odom()
                self.imu_data()
                self.get_logger().info(f"en1 = {self.en1}, en2 = {self.en2}, angle = {self.angle}")
            else:
                self.buff_data[self.count] = self.rec_data
                self.count += 1
            #self.get_logger().info(f"[receive] angle: {self.angle}, en1: {self.en1}, en2: {self.en2}")

    def imu_data(self):
        current_time = time.time()
        dt = current_time - self.pre_measurement_time_imu
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "base_link"
        imu_msg.orientation = self.create_quaternion_msg_from_yaw(self.angle * self.rad)
        imu_msg.angular_velocity.z = ((self.angle - self.pre_angle) * self.rad) / dt
        self.imu_pub.publish(imu_msg)
        self.pre_measurement_time_imu = current_time
        self.pre_angle = self.angle
        #self.get_logger().info(f"[IMU PUB] theta = {self.angle * self.rad}")

    def odom(self):
        current_time = time.time()
        dt = current_time - self.pre_measurement_time
        omega_right = self.en1 * self.scaling_factor
        omega_left = self.en2 * self.scaling_factor
        delta_theta = (omega_right + omega_left) / self.wheels_distance
        delta_s = (omega_right - omega_left) / 2.0
        self.get_logger().info(f"[omega_left={self.en1 * self.scaling_factor}, omega_right={self.en2 * self.scaling_factor}")

        self.theta = self.pre_theta + delta_theta
        self.x = self.pre_x + math.cos(self.theta) * delta_s
        self.y = self.pre_y + math.sin(self.theta) * delta_s

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.fixed_frame_odom
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = self.create_quaternion_msg_from_yaw(self.theta)
        odom_msg.twist.twist.linear.x = delta_s / dt
        odom_msg.twist.twist.angular.z = delta_theta / dt
        self.odom_pub.publish(odom_msg)

        self.pre_theta = self.theta
        self.pre_x = self.x
        self.pre_y = self.y
        self.pre_measurement_time = current_time
        self.get_logger().info(f"[ODOM PUB] x={self.x:.2f}, y={self.y:.2f}")

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