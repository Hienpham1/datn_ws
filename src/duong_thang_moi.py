import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from std_msgs.msg import Bool, Int16MultiArray
import math


def euler_to_quaternion(yaw):
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0)
    )

def apply_offset(x, y, theta, offset=0.0):
    x_r = x + offset * math.cos(theta)
    y_r = y + offset * math.sin(theta)
    return x_r, y_r, theta


class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner')

        self.pose_pub = self.create_publisher(PoseArray, '/planned_path', 10)
        self.spray_pub = self.create_publisher(Int16MultiArray, '/topic_paint', 10)
        self.wp_idx = 0
        self.timer_period = 0.2 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Danh sách waypoint: (x, y, theta)
        self.waypoints = [
            (1.0, 0.0, 0.0),
            #(2.0, 0.0, math.pi/2),
            #(1.5, 0.245, math.pi/2),
        ]

        # Lịch bật/tắt sơn (theo chỉ số waypoint)
        self.paint_schedule = {
            0: 1,  # bật sơn trước khi đi đến WP0
            1: 0,  # tắt sơn sau khi đến WP0
            2: 1,  # bật lại trước khi đi đến WP2
            3: 0   # tắt sau khi đến WP2
        }

        self.create_subscription(Bool, '/wp_reached', self.wp_reached_callback, 10)

        self.get_logger().info('Trajectory Planner started...')
        self.send_current_waypoint()  # Gửi WP đầu tiên ngay khi khởi động

    def wp_reached_callback(self, msg):
        if msg.data:
            self.wp_idx += 1
            if self.wp_idx >= len(self.waypoints):
                self.get_logger().info("Hoàn thành tất cả các waypoint.")
                self.spray_pub.publish(Int16MultiArray(data=[0]))  # Tắt sơn


    def send_current_waypoint(self):
        x, y, theta = self.waypoints[self.wp_idx]
        x, y, theta = apply_offset(x, y, theta)

        pose_array = PoseArray()
        pose_array.header.frame_id = 'odom'
        pose_array.header.stamp = self.get_clock().now().to_msg()

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation = euler_to_quaternion(theta)
        pose_array.poses.append(pose)

        # Gửi waypoint hiện tại
        self.pose_pub.publish(pose_array)

        # Sơn nếu tại WP này có cấu hình
        if self.wp_idx in self.paint_schedule:
            paint_cmd = self.paint_schedule[self.wp_idx]
            self.spray_pub.publish(Int16MultiArray(data=[paint_cmd]))
            state = "BẬT" if paint_cmd else "TẮT"
            self.get_logger().info(f"Sơn: {state} (tại WP{self.wp_idx})")

        self.get_logger().info(
            f"Gửi WP{self.wp_idx}: ({x:.2f}, {y:.2f}, {math.degrees(theta):.1f}°)"
    )


    def timer_callback(self):
        if self.wp_idx < len(self.waypoints):
            self.send_current_waypoint()
        else:
            self.timer.cancel()
            self.get_logger().info("Đã hoàn thành tất cả waypoint.")
            self.spray_pub.publish(Int16MultiArray(data=[0]))  # đảm bảo tắt sơn



def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
