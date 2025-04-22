#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

class PIDInputNode(Node):
    def __init__(self):
        super().__init__('pid_input_node')
        self.publisher_ = self.create_publisher(Int16MultiArray, 'pid_values', 10)
        self.timer = self.create_timer(5.0, self.timer_callback)  # mỗi 5s hỏi 1 lần

    def timer_callback(self):
        try:
            inp = input("Nhập Kp Ki Kd cho bánh trái và phải (vd: 13 0 0 13 0 0): ")
            values = list(map(float, inp.strip().split()))
            if len(values) != 6:
                self.get_logger().error("Vui lòng nhập đúng 6 giá trị!")
                return
            
            # Scale nếu cần (nếu truyền float không được qua serial)
            scaled_values = [int(v * 100) for v in values]  # nhân 100 để lấy phần thập phân

            msg = Int16MultiArray()
            msg.data = scaled_values
            self.publisher_.publish(msg)
            self.get_logger().info(f"Đã gửi PID: {scaled_values}")
        except Exception as e:
            self.get_logger().error(f"Lỗi khi nhập: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PIDInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

