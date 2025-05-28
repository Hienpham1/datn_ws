#https://github.com/YJ0528/minibot?tab=readme-ov-file
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Twist
from nav_msgs.msg import Odometry
import math

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Tham số: điều chỉnh theo loại đường
        self.declare_parameter('lookahead_distance', 0.25)  # đổi tại launch nếu cần
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value

        # ROS I/O
        self.path_sub = self.create_subscription(PoseArray, '/planned_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom_encoder', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Biến lưu trạng thái
        self.path = []
        self.current_pose = None
        self.integral_angle = 0.0
        self.prev_angle_error = 0.0

        self.get_logger().info(f"Pure Pursuit Node started. Lookahead = {self.lookahead_distance} m")

    def path_callback(self, msg):
        self.path = msg.poses
        self.integral_angle = 0.0
        self.prev_angle_error = 0.0
        self.get_logger().info(f"Received path with {len(self.path)} waypoints.")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.compute_control()

    def compute_control(self):
        if not self.path or self.current_pose is None:
            return

        rx = self.current_pose.position.x
        ry = self.current_pose.position.y

        # Tìm điểm lookahead
        lookahead_point = None
        for pose in self.path:
            dx = pose.position.x - rx
            dy = pose.position.y - ry
            distance = math.hypot(dx, dy)
            if distance >= self.lookahead_distance:
                lookahead_point = pose
                break

        if lookahead_point is None:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            self.get_logger().info("Reached final waypoint. Stopping.")
            return

        # Góc tới điểm
        dx = lookahead_point.position.x - rx
        dy = lookahead_point.position.y - ry
        angle_to_target = math.atan2(dy, dx)

        # Góc hiện tại
        q = self.current_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Sai số góc
        angle_error = math.atan2(math.sin(angle_to_target - current_yaw),
                                 math.cos(angle_to_target - current_yaw))
        self.integral_angle += angle_error
        derivative_angle = angle_error - self.prev_angle_error
        self.prev_angle_error = angle_error
        # PID gains (tune thử nghiệm)
        Kp, Ki, Kd = 1.2, 0.01, 0.1  
        w = Kp * angle_error + Ki * self.integral_angle + Kd * derivative_angle
        w = max(-0.5, min(0.5, w))
        
        # Điều khiển vận tốc
        lookahead_dist = math.hypot(dx, dy)
        #v = max(0.1, min(0.25, 0.5 * lookahead_dist))         # tốc độ tuyến tính giảm nếu gần
        v = 0.2 * (1 + math.tanh(lookahead_dist - self.lookahead_distance))

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"[CONTROL] dist={lookahead_dist:.2f}m, angle_err={math.degrees(angle_error):.1f}°, v={v:.2f}, w={w:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
