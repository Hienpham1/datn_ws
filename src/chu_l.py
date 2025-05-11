import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from std_msgs.msg import Int32
import math


def euler_to_quaternion(yaw):
    """ Chuyển yaw (rad) → quaternion ROS """
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0)
    )


def apply_offset(x, y, theta, offset=0.245):
    """Chuyển từ pose đầu vẽ ➜ pose của robot (ngược lại offset)"""
    x_r = x - offset * math.cos(theta)
    y_r = y - offset * math.sin(theta)
    return x_r, y_r, theta


class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner_node')

        self.pose_pub = self.create_publisher(PoseArray, '/planned_path', 10)
        self.spray_pub = self.create_publisher(Int32, '/son', 10)

        self.timer = self.create_timer(1.0, self.publish_path)

        self.get_logger().info('🧭 Trajectory Planner node started...')

    def publish_path(self):
        pose_array = PoseArray()
        pose_array.header.frame_id = 'map'
        pose_array.header.stamp = self.get_clock().now().to_msg()

        # 🛠️ === Vẽ hình chữ L ===
        waypoints = []

        # ➊ Di chuyển robot để đầu vẽ nằm tại (0, 0, 0)
        start_x, start_y, start_theta = apply_offset(0.0, 0.0, 0.0)
        waypoints.append((start_x, start_y, start_theta))
        self.spray_pub.publish(Int32(data=0))  # tắt cơ cấu vẽ

        # ➋ Bật cơ cấu vẽ tại (0, 0)
        self.spray_pub.publish(Int32(data=1))  # bật sơn

        # ➌ Vẽ đường thẳng từ (0,0) → (2,0)
        x0, y0 = 0.0, 0.0
        for i in range(20):  # chia nhỏ đoạn 2m
            x = x0 + 0.1 * i
            y = y0
            theta = 0.0
            xr, yr, th = apply_offset(x, y, theta)
            waypoints.append((xr, yr, th))

        # ➍ Tắt bơm tại điểm (2, 0)
        self.spray_pub.publish(Int32(data=0))  # tắt sơn

        # ➎ Robot chạy đến điểm chuẩn bị quay trái (2.0, 0.5)
        xr, yr, th = apply_offset(2.0, 0.5, math.pi / 2)
        waypoints.append((xr, yr, th))

        # ➏ Lui robot sao cho đầu vẽ trở về (2, 0.245)
        xr, yr, th = apply_offset(2.0, 0.245, math.pi / 2)
        waypoints.append((xr, yr, th))

        # ➐ Bật bơm lại
        self.spray_pub.publish(Int32(data=1))

        # ➑ Vẽ đường dọc từ (2, 0.245) đến (2, 2)
        y_start = 0.245
        for i in range(20):
            x = 2.0
            y = y_start + i * 0.08775  # (2 - 0.245) / 20
            theta = math.pi / 2
            xr, yr, th = apply_offset(x, y, theta)
            waypoints.append((xr, yr, th))

        # ➒ Tắt sơn khi hoàn tất
        self.spray_pub.publish(Int32(data=0))

        # 🧱 Tạo PoseArray
        for x, y, yaw in waypoints:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.orientation = euler_to_quaternion(yaw)
            pose_array.poses.append(pose)

        self.pose_pub.publish(pose_array)
        self.get_logger().info(f"✅ Published path with {len(pose_array.poses)} poses for chữ L.")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
