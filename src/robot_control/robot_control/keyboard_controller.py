import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)  # Xuất bản lên topic cmd_vel
        self.get_logger().info("Keyboard controller started. Use W/A/S/D to move, Q to quit.")

    def get_key(self):
        """
        Đọc phím từ bàn phím (chế độ không chặn).
        """
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def run(self):
        """
        Vòng lặp chính để đọc phím và gửi lệnh vận tốc.
        """
        try:
            while True:
                key = self.get_key()
                msg = Twist()

                # Điều khiển bằng các phím W/A/S/D
                if key == 'w':  # Tiến
                    msg.linear.x = 0.2
                elif key == 's':  # Lùi
                    msg.linear.x = -0.2
                elif key == 'a':  # Quay trái
                    msg.angular.z = 0.2
                elif key == 'd':  # Quay phải
                    msg.angular.z = -0.2
                elif key == 'q':  # Thoát
                    self.get_logger().info("Exiting keyboard controller.")
                    break
                else:
                    # Dừng robot nếu không nhấn phím hợp lệ
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0

                # Xuất bản lệnh vận tốc
                self.publisher.publish(msg)
                self.get_logger().info(f"Published cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}")

        except KeyboardInterrupt:
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.publisher.publish(stop_msg)
            self.get_logger().info("stop motor")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
