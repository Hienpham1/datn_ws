import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray

class FanShapePainter(Node):
    def __init__(self):
        super().__init__('fan_shape_painter')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.paint_pub = self.create_publisher(Int16MultiArray, '/topic_paint', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.v_straight = 0.25
        self.w_turn = 0.5
        self.t_straight = 10.0
        self.t_turn = 7.85

        # For arc back to (0,0)
        self.v_arc = 0.25
        self.w_arc = 0.088
        self.t_arc = 27.0

        self.states = [
            {'type': 'straight', 'duration': self.t_straight, 'paint_on': (2, 10)},
            {'type': 'turn', 'duration': self.t_turn},
            {'type': 'straight', 'duration': self.t_straight, 'paint_on': (0, 8)},
            {'type': 'arc', 'duration': self.t_arc, 'v': self.v_arc, 'w': self.w_arc, 'paint_on': (0, self.t_arc)}
        ]

        self.current_state_index = 0
        self.state_start_time = self.get_clock().now().seconds_nanoseconds()[0] + \
                                 self.get_clock().now().seconds_nanoseconds()[1] * 1e-9

        self.get_logger().info("Fan Shape Painter Node Started")

    def control_loop(self):
        if self.current_state_index >= len(self.states):
            self.cmd_pub.publish(Twist())
            self.send_paint_command([0, 0])
            self.get_logger().info("Completed fan shape.")
            self.timer.cancel()
            return

        now = self.get_clock().now().seconds_nanoseconds()[0] + \
              self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
        elapsed = now - self.state_start_time

        state = self.states[self.current_state_index]
        cmd = Twist()

        if state['type'] == 'straight':
            cmd.linear.x = self.v_straight
            cmd.angular.z = 0.0
            paint_start, paint_end = state['paint_on']
            if paint_start <= elapsed <= paint_end:
                self.send_paint_command([40, 40])
            else:
                self.send_paint_command([0, 0])

        elif state['type'] == 'turn':
            cmd.linear.x = self.v_straight
            cmd.angular.z = self.w_turn
            self.send_paint_command([0, 0])

        elif state['type'] == 'arc':
            cmd.linear.x = state['v']
            cmd.angular.z = state['w']
            paint_start, paint_end = state['paint_on']
            if paint_start <= elapsed <= paint_end:
                self.send_paint_command([40, 40])
            else:
                self.send_paint_command([0, 0])

        self.cmd_pub.publish(cmd)

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
    node = FanShapePainter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
