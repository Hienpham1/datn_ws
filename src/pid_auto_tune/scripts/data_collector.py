import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry
import numpy as np
import os
from builtin_interfaces.msg import Time

class DataCollectorNode(Node):
    def __init__(self):
        super().__init__('pid_data_collector')

        # Khởi tạo buffer lưu dữ liệu
        self.path_data = None
        self.odom_data = []
        self.cmd_vel_data = []
        self.timestamps = []

        # Subscriptions
        self.create_subscription(PoseArray, '/planned_path', self.path_callback, 10)
        self.create_subscription(Odometry, '/odom_encoder', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        self.get_logger().info('Data Collector Node started. Listening to rosbag...')

    def path_callback(self, msg):
        self.path_data = msg.poses
        self.get_logger().info(f"Received path with {len(self.path_data)} poses.")

    def odom_callback(self, msg):
        pose = msg.pose.pose
        time = msg.header.stamp
        self.odom_data.append([
            time.sec + time.nanosec * 1e-9,
            pose.position.x,
            pose.position.y
        ])

    def cmd_callback(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.cmd_vel_data.append([
            now,
            msg.linear.x,
            msg.angular.z
        ])

    def save_data(self):
        save_path = os.path.join(
            os.path.dirname(__file__), '..', 'data', 'data_log.npz'
        )
        os.makedirs(os.path.dirname(save_path), exist_ok=True)

        np.savez_compressed(save_path,
                            path=np.array([[p.position.x, p.position.y] for p in self.path_data]) if self.path_data else [],
                            odom=np.array(self.odom_data),
                            cmd_vel=np.array(self.cmd_vel_data))
        self.get_logger().info(f"Saved data to {save_path}")


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt detected. Saving data...')
        node.save_data()
    finally:
        node.destroy_node()
        rclpy.shutdown()
