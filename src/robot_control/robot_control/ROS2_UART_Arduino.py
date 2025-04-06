import rclpy
from rclpy.node import Node
#from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import serial
import math
import time

class Arduino(Node):
    def __init__(self):
        super().__init__('arduino_node')

        # Declare and get parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('topic_odom', 'odom_encoder')
        self.declare_parameter('topic_imu', 'IMU')
        self.declare_parameter('fixed_frame_odom', 'odom')
        self.declare_parameter('topic_velocities', 'cmd_vel')
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheels_distance', 0.3)
        self.declare_parameter('pulses_per_revolution', 2000.0)

        self.port = self.get_parameter('port').value
        self.topic_odom = self.get_parameter('topic_odom').value
        self.topic_imu = self.get_parameter('topic_imu').value
        self.fixed_frame_odom = self.get_parameter('fixed_frame_odom').value
        self.topic_velocities = self.get_parameter('topic_velocities').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheels_distance = self.get_parameter('wheels_distance').value
        self.pulses_per_revolution = self.get_parameter('pulses_per_revolution').value

        # Initialize publishers and subscribers
        self.odom_pub = self.create_publisher(Odometry, self.topic_odom, 10)
        self.imu_pub = self.create_publisher(Imu, self.topic_imu, 10)
        #self.velocities_sub = self.create_subscription(
        #    Int16MultiArray, self.topic_velocities, self.velocities_callback, 10)
        self.velocities_sub = self.create_subscription(
            Twist, self.topic_velocities, self.velocities_callback,10)

        # Variables
        self.ser = None
        self.rec_data = 0
        self.count = 0
        self.buff_data = [0] * 8
        self.angle = 0
        self.en1 = 0
        self.en2 = 0
        self.pre_angle = 0
        self.delta_theta = 0
        self.delta_s = 0
        self.omega_right = 0
        self.omega_left = 0
        self.theta = 0
        self.x = 0
        self.y = 0
        self.pre_theta = 0
        self.pre_x = 0
        self.pre_y = 0
        self.scaling_factor = 0
        self.odom_angle = 0
        self._check = False

        # Connect to serial port
        self.connect_serial()

        # Initialize timer
        self.timer = self.create_timer(0.01, self.read_serial)

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, baudrate=115200, timeout=1)
            self.get_logger().info(f"Serial port {self.port} initialized and opened.")
        except serial.SerialException as e:
            self.get_logger().error(f"Unable to open serial port {self.port}. Retrying in 5 seconds...")
            time.sleep(5)
            self.connect_serial()
    def read_serial(self):
        if self.ser.in_waiting > 0:
            self.rec_data = self.ser.read(1)[0]
            if self.count == 0 and self.rec_data != 0x02:
                return
            if self.count == 7 and self.rec_data != 0x03:
                return
            if self.count == 7 and self.rec_data == 0x03:
                self.angle = (self.buff_data[1] << 8) | self.buff_data[2]
                self.en1 = (self.buff_data[3] << 8) | self.buff_data[4]
                self.en2 = (self.buff_data[5] << 8) | self.buff_data[6]
                self.count = 0
                if not self._check:
                    if self.angle != 0:
                        return
                    else:
                        self._check = True
                self.odom()
                self.imu_data()
                self.get_logger().info(f"en1 = {self.en1}, en2 = {self.en2}, angle = {self.angle}")
            else:
                self.buff_data[self.count] = self.rec_data
                self.count += 1
    def velocities_callback(self, msg):
        """
        Callback xử lý dữ liệu từ topic_velocities (cmd_vel).
        Tính toán tốc độ bánh xe từ vận tốc tuyến tính và góc, sau đó gửi đến Arduino.
        """
        self.get_logger().info("------velo callback bat dau")
        # Lấy vận tốc tuyến tính và góc từ msg
        linear_x = msg.linear.x  # Vận tốc tuyến tính (m/s)
        angular_z = msg.angular.z  # Vận tốc góc (rad/s)
    
        # Tính toán tốc độ bánh xe (trái và phải)
        left_speed = (linear_x - angular_z * self.wheels_distance / 2)
        right_speed = (linear_x + angular_z * self.wheels_distance / 2)
    
        # Chuyển đổi tốc độ từ rad/s sang giá trị PWM (giới hạn từ -255 đến 255)
        kpwm = 100
        max_pwm = 255
        left_pwm = int(max(-max_pwm, min(max_pwm, left_speed * kpwm)))
        right_pwm = int(max(-max_pwm, min(max_pwm, right_speed * kpwm)))
    
        # Gửi lệnh điều khiển đến Arduino
        self.write_serial(0, left_pwm, right_pwm)
    
        # Ghi log thông tin
        self.get_logger().info(f"Received cmd_vel: linear.x={linear_x}, angular.z={angular_z}")
        self.get_logger().info(f"Calculated PWM: left={left_pwm}, right={right_pwm}")
    def delta(self):
        self.scaling_factor = (2.0 * math.pi * self.wheel_radius) / self.pulses_per_revolution
        self.omega_right = self.en1 * self.scaling_factor
        self.omega_left = self.en2 * self.scaling_factor
        self.delta_theta = (self.omega_right + self.omega_left) / self.wheels_distance
        self.delta_s = (self.omega_right - self.omega_left) / 2.0

    def odom(self):
        self.delta()
        self.theta = self.pre_theta + self.delta_theta
        self.odom_angle = self.theta * (180 / math.pi) * 10
        self.x = self.pre_x + (math.cos(self.theta) * self.delta_s)
        self.y = self.pre_y + (math.sin(self.theta) * self.delta_s)
        self.pre_theta = self.theta
        self.pre_x = self.x
        self.pre_y = self.y
    
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.fixed_frame_odom
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = Quaternion()
        self.odom_pub.publish(odom_msg)
    def write_serial(self, byte_reset, v_banh_trai, v_banh_phai):
        dir_trai = 1 if v_banh_trai < 0 else 0
        dir_phai = 1 if v_banh_phai < 0 else 0
        v_banh_trai = abs(v_banh_trai)
        v_banh_phai = abs(v_banh_phai)
        v_banh_trai = min(v_banh_trai, 255)
        v_banh_phai = min(v_banh_phai, 255)
    
        chieu_trai = (dir_trai << 7) | 0x01
        chieu_phai = (dir_phai << 7) | 0x02
    
        command = f", {byte_reset} {chieu_trai} {v_banh_trai} {chieu_phai} {v_banh_phai}."
        self.ser.write(command.encode())
        self.get_logger().info(f"command serial: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = Arduino()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
