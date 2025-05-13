import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from std_msgs.msg import Int32
import math


def euler_to_quaternion(yaw):
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0)
    )

def apply_offset(x, y, theta, offset=0.245):
    x_r = x - offset * math.cos(theta)
    y_r = y - offset * math.sin(theta)
    return x_r, y_r, theta

class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner')

        self.pose_pub = self.create_publisher(PoseArray, '/planned_path', 10)
        self.spray_pub = self.create_publisher(Int32, '/son', 10)

        self.timer = self.create_timer(1.0, self.publish_path)

        self.get_logger().info('Trajectory Planner started...')

        # State variables
        self.path = []
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_theta = 0.0

    def spray_on(self):
        self.get_logger().info("Bật phun sơn")
        self.spray_pub.publish(Int32(data=1))

    def spray_off(self):
        self.get_logger().info("Tắt phun sơn")
        self.spray_pub.publish(Int32(data=0))

    def move_straight(self, distance: float, step: float = 0.1):
        steps = int(abs(distance) / step)
        direction = 1 if distance >= 0 else -1
        for i in range(1, steps + 1):
            dx = direction * step * i * math.cos(self.curr_theta)
            dy = direction * step * i * math.sin(self.curr_theta)
            x = self.curr_x + dx
            y = self.curr_y + dy
            self._add_pose(x, y, self.curr_theta)

        # Cập nhật vị trí hiện tại
        self.curr_x += direction * distance * math.cos(self.curr_theta)
        self.curr_y += direction * distance * math.sin(self.curr_theta)

    def rotate_in_place(self, angle_rad: float, steps=10):
        delta = angle_rad / steps
        for i in range(1, steps + 1):
            new_theta = self.curr_theta + delta * i
            self._add_pose(self.curr_x, self.curr_y, new_theta)

        self.curr_theta = self._normalize_angle(self.curr_theta + angle_rad)

    def arc(self, radius: float, angle_rad: float, steps=20):
        cx = self.curr_x - radius * math.sin(self.curr_theta)
        cy = self.curr_y + radius * math.cos(self.curr_theta)

        for i in range(1, steps + 1):
            delta = angle_rad * i / steps
            new_theta = self.curr_theta + delta
            x = cx + radius * math.sin(new_theta)
            y = cy - radius * math.cos(new_theta)
            self._add_pose(x, y, new_theta)

        self.curr_x = cx + radius * math.sin(self.curr_theta + angle_rad)
        self.curr_y = cy - radius * math.cos(self.curr_theta + angle_rad)
        self.curr_theta = self._normalize_angle(self.curr_theta + angle_rad)

    def _add_pose(self, x, y, yaw):
        pose = Pose()
        pose.position.x, pose.position.y, _ = apply_offset(x, y, yaw)
        pose.orientation = euler_to_quaternion(yaw)
        self.path.append(pose)

    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def publish_path(self):
        # Reset path
        self.path = []
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_theta = 0.0

        self.spray_off()
        self._add_pose(self.curr_x, self.curr_y, self.curr_theta)

        ###  Viết kế hoạch ở đây:
        self.move_straight(2.0)
        self.spray_on()

        self.rotate_in_place(math.pi / 2)
        self.move_straight(2.0)

        self.spray_off()
        self.rotate_in_place(-math.pi / 2)
        self.arc(radius=1.0, angle_rad=math.pi / 2)
        self.spray_on()

        ### có thể thêm bất kỳ lệnh nào ở đây

        # Gửi PoseArray
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poses = self.path
        self.pose_pub.publish(msg)

        self.get_logger().info(f"Đã gửi đường đi với {len(self.path)} điểm.")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
