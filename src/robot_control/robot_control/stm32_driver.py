import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import Int32 #msgs de pub du lieu encoder
import serial
from geometry_msgs.msg import Twist


#dinh nghia giao thuc giao tiep voi stm32
START_BYTE = 0xFF
CMD_MOTOR = 0x01
CMD_ENCODER = 0x02

class STM32Driver(Node):
    def __init__(self):
        super().__init__('stm32_driver')
        #khoi tao cong serial de giao tiep voi stm32
        # cac parameters cua giao thuc
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        self.serial_port = serial.Serial(
            port=port, #tim cong cho dung
            baudrate=baud,
            timeout=1
        )
        self._init_publishers()
        self._init_subscribers()

        #tao publisher cho encoder
        self.encoder_pub = self.create_publisher(Int32, 'encoder_values', 10)

        #tao subscriber de dieu khien dong co
        self.velocity_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10
        )

        #tao timer de doc du lieu stm32
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info('stm32 drvire node start')

    def _init_publishers(self):
        self.left_encoder_pub = self.create_publisher(Int32, 'left_encoder', 10)
        self.right_encoder_pub = self.create_publisher(Int32, 'right_encoder', 10)
        self.battery_pub = self.create_publisher(Int32, 'battery_voltage', 10)

    def _init_subscribers(self):
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10
        )
        self.cmd_vel_sub

    def timer_callback(self):
        try:
            if self.serial_port.in_waiting >=6: #nho nhat 
                if self.serial_port.read(1)[0] == START_BYTE:
                    cmd = self.serial_port.read(1)[0]
                    if cmd == CMD_ENCODER:
                        #doc gia tri encoder
                        left_enc = int.from_bytes(
                            self.serial_port.read(2),
                            'little',
                            signed=True
                        )
                        right_enc = int.from_bytes(
                            self.serial_port.read(2),
                            'little',
                            signed=True
                        )
                        #sau do pub gia tri encoder
                        self.publish_encoder_values(left_enc, right_enc)
        except Exception as e:
            self.get_logger().error(f"Serial read err: {str(e)}")
            

    def velocity_callback(self, msg):
        wheel_base = 0.3
        wheel_radius = 0.05 
        gear_ratio = 2
        # toc do tinh tien x va toc do goc z thanh toc do 2 banh
        # nhan cho 2 vi bo truyen co ti so truyen la 2
        left_speed = 2 * (msg.linear.x - msg.angular.z * wheel_base/2) / wheel_radius
        right_speed = 2 * (msg.linear.x + msg.angular.z * wheel_base/2) / wheel_radius

        #chuyen doi tu m/s sang PWM
        max_pwm = 1000
        left_pwm = int(left_speed * 100) # hang so khech dai
        right_pwm = int(right_speed * 100)

        left_pwm = max(-max_pwm, min(max_pwm, left_pwm))
        right_pwm = max(-max_pwm, min(max_pwm, right_pwm))

        #sau do gui lenh cho stm32
        self.send_motor_command(left_pwm, right_pwm)
        self.last_cmd_time = self.get_clock().now()

    def send_motor_command(self, left, right):
        try:
            #giao thuc truyen
            #[START_BYTE]{CMD_MOTOR}[left_speed][right_speed]
            command = bytearray([START_BYTE, CMD_MOTOR])
            command.extend(left.to_bytes(2, 'little', signed=True))
            command.extend(right.to_bytes(2, 'little', signed=True))
            self.serial_port.write(command)

            self.get_logger().debug(f'sent motor command: L={left}, R={right}')
        except serial.SerialException as e:
            self.get_logger().error(f'serial comute err: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    try:
        node = STM32Driver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        rclpy.logging.get_logger('stm32_driver').error(f'Node err: {str(e)}')
        #node.get_logger().error(f'Node err: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()