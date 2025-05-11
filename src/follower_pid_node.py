import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
import numpy as np
import math

class FollowerPIDNode(Node):
    def __init__(self):
        super().__init__('follower_pid_node')

        # Tham số PID
        self.Kp = 2.0
        self.Ki = 0.0
        self.Kd = 0.2
        self.v = 0.3  # tốc độ tuyến tính cố định
        self.dt = 0.1
        self.arrive_thresh = 0.15  # m

        # Trạng thái PID
        self.E = 0.0
        self.old_e = 0.0

        # Danh sách quỹ đạo
        self.waypoints = []
        self.current_waypoint_idx = 0

        # Vị trí hiện tại
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_theta = 0.0

        # Subscriber
        self.create_subscription(PoseArray, '/planned_path', self.path_callback, 10)
        self.create_subscription(Odometry, '/odom_encoder', self.odom_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer loop
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("🧠 Follower PID Node Started...")

    def path_callback(self, msg):
        self.waypoints = msg.poses
        self.current_waypoint_idx = 0
        self.get_logger().info(f"📥 Received {len(self.waypoints)} waypoints.")

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.curr_x = pose.position.x
        self.curr_y = pose.position.y

        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.curr_theta = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if self.current_waypoint_idx >= len(self.waypoints):
            self.stop_robot()
            return

        goal_pose = self.waypoints[self.current_waypoint_idx]
        gx = goal_pose.position.x
        gy = goal_pose.position.y

        dx = gx - self.curr_x
        dy = gy - self.curr_y
        dist = math.hypot(dx, dy)

        if dist < self.arrive_thresh:
            self.get_logger().info(f"✅ Reached waypoint {self.current_waypoint_idx + 1}")
            self.current_waypoint_idx += 1
            self.E = 0
            self.old_e = 0
            return

        # Tính góc đến mục tiêu
        goal_theta = math.atan2(dy, dx)
        alpha = self.fix_angle(goal_theta - self.curr_theta)

        # PID
        e_P = alpha
        e_I = self.E + e_P
        e_D = e_P - self.old_e
        self.E = e_I
        self.old_e = e_P

        w = self.Kp * e_P + self.Ki * e_I + self.Kd * e_D
        w = self.fix_angle(w)

        # Xuất Twist
        cmd = Twist()
        cmd.linear.x = self.v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def fix_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def stop_robot(self):
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        self.cmd_pub.publish(stop)

def main(args=None):
    rclpy.init(args=args)
    node = FollowerPIDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
