import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
import time


class SquarePainter(Node):
    def __init__(self):
        super().__init__('square_painter')

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.paint_pub = self.create_publisher(Int16MultiArray, '/topic_paint', 10)

        # Timer 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        # Các thông số chuyển động
        self.v = 0.25
        self.w = 0.5
        self.straight_time = 10.0
        self.paint_start = 2.0
        self.paint_end = 10.0
        self.turn_time = 7.85

        # Danh sách các trạng thái chuyển động
        self.states = [
            {'type': 'straight', 'duration': self.straight_time, 'paint_on': (self.paint_start, self.paint_end)},
            {'type': 'turn', 'duration': self.turn_time},
            {'type': 'straight', 'duration': self.straight_time, 'paint_on': (0, 8)},
            {'type': 'turn', 'duration': self.turn_time},
            {'type': 'straight', 'duration': self.straight_time, 'paint_on': (0, 8)},
            {'type': 'turn', 'duration': self.turn_time},
            {'type': 'straight', 'duration': self.straight_time, 'paint_on': (0, 8)},
            {'type': 'turn', 'duration': self.turn_time},
        ]

        self.current_state_index = 0
        self.state_start_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        self.get_logger().info("Square Painter Node Started")

    def control_loop(self):
        if self.current_state_index >= len(self.states):
            self.cmd_pub.publish(Twist())
            self.send_paint_command([0, 0])
            self.get_logger().info("Completed square.")
            self.timer.cancel()
            return

        now = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        elapsed = now - self.state_start_time

        state = self.states[self.current_state_index]

        # Gửi lệnh chuyển động
        cmd = Twist()
        if state['type'] == 'straight':
            cmd.linear.x = self.v
            cmd.angular.z = 0.0
            # Bật tắt bơm
            paint_start, paint_end = state['paint_on']
            if paint_start <= elapsed <= paint_end:
                self.send_paint_command([40, 40])
            else:
                self.send_paint_command([0, 0])

        elif state['type'] == 'turn':
            cmd.linear.x = self.v
            cmd.angular.z = self.w
            self.send_paint_command([0, 0])  # không sơn khi quay

        self.cmd_pub.publish(cmd)

        # Chuyển sang trạng thái tiếp theo nếu hết thời gian
        if elapsed >= state['duration']:
            self.current_state_index += 1
            self.state_start_time = now
            self.get_logger().info(f"Chuyển sang bước {self.current_state_index}")

    def send_paint_command(self, data):
        msg = Int16MultiArray()
        msg.data = data
        self.paint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SquarePainter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
