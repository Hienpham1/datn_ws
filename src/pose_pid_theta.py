import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
import math
import numpy as np
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32MultiArray


class FollowerPIDNode(Node):
    def __init__(self):
        super().__init__('follower_pid_node')

        # tham so PID
        self.declare_parameter("Kp", 0.5)
        self.declare_parameter("Ki", 0.01)
        self.declare_parameter("Kd", 0.0)
        self.declare_parameter("v", 0.3)

        self.Kp = self.get_parameter("Kp").value
        self.Ki = self.get_parameter("Ki").value
        self.Kd = self.get_parameter("Kd").value
        self.v = self.get_parameter("v").value

        self.dt = 0.05
        self.arrive_thresh = 0.02  # ngưỡng đến điểm (mét)
        self.slow_down_dist = 0.5  # ngưỡng giảm 5tốc (mét)

        # bien tich phan va vi phan
        self.E = 0.0
        self.old_e = 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        # danh sach quy dao
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_theta = 0.0
        self.desir_theta = 0.0

        # Subscribers
        self.create_subscription(PoseArray, '/planned_path', self.path_callback, 10)
        self.create_subscription(Odometry, '/odom_encoder', self.odom_callback, 10)
        self.create_subscription(Float32MultiArray, '/pid_params_theta', self.update_pid_theta, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_desired_theta = self.create_publisher(Float32MultiArray, 'desired_theta', 10)
        self.pub_current_theta = self.create_publisher(Float32MultiArray, 'current_theta', 10)

        # Timer
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("Follower PID Node Started...")

    def update_pid_theta(self, msg):
        kp, ki, kd = msg.data
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.get_logger().info(f"Updated PID: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}")

    def path_callback(self, msg):
        self.waypoints = msg.poses
        self.current_waypoint_idx = 0
        self.E = 0.0
        self.old_e = 0.0
        self.get_logger().info(f"[PATH] Robot at: ({self.curr_x:.3f}, {self.curr_y:.3f})")
        self.get_logger().info(f"Received {len(self.waypoints)} waypoints.")

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.curr_x = pose.position.x
        self.curr_y = pose.position.y

        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.curr_theta = math.atan2(siny_cosp, cosy_cosp)

        current_msg = Float32MultiArray()
        current_msg.data = [self.curr_theta]
        self.pub_current_theta.publish(current_msg)

    def control_loop(self):
        if not self.waypoints or self.current_waypoint_idx >= len(self.waypoints):
            self.stop_robot()
            return

        goal_pose = self.waypoints[self.current_waypoint_idx]
        gx = goal_pose.position.x
        gy = goal_pose.position.y

        dx = gx - self.curr_x
        dy = gy - self.curr_y
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            goal_theta = self.curr_theta  # giữ nguyên hướng hiện tại
        else:
            goal_theta = math.atan2(dy, dx)
        self.get_logger().info(f"dx={dx:.3f}, dy={dy:.3f}, goal_theta_deg={math.degrees(goal_theta):.2f}")
        self.get_logger().info(f"[POS] curr_x={self.curr_x:.3f}, curr_y={self.curr_y:.3f}, curr_theta_deg={math.degrees(self.curr_theta):.2f}")
        dist = math.sqrt(dx**2 + dy**2)
        # bỏ xuống sau khi tính alpha
        if dist < self.arrive_thresh and abs(alpha) < angle_thresh:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_idx + 1}")
            self.current_waypoint_idx += 1
            self.E = 0.0
            self.old_e = 0.0
            return

        # tinh goc mong muon
        goal_theta = math.atan2(dy, dx)
        self.desir_theta = goal_theta

        desired_msg = Float32MultiArray()
        desired_msg.data = [self.desir_theta]
        self.pub_desired_theta.publish(desired_msg)

        alpha = self.fix_angle(goal_theta - self.curr_theta)
        angle_thresh = math.radians(8)
        self.get_logger().info(f"[DEBUG] Goal Theta: {math.degrees(goal_theta):.2f}, "
                       f"Current Theta: {math.degrees(self.curr_theta):.2f}, "
                       f"Alpha: {math.degrees(alpha):.2f}")
        # tinh PID
        e_P = alpha
        e_I = self.E + e_P
        e_D = e_P - self.old_e

        self.E = e_I
        self.old_e = e_P

        w = self.Kp * e_P + self.Ki * e_I * self.dt + self.Kd * e_D / self.dt
        w = np.clip(w, -0.3, 0.3)  # gioi han

        if abs(alpha) > angle_thresh:
            v = 0.0  # đứng lại, chỉ xoay
        else:
            v = self.v
            if dist < self.slow_down_dist:
                v = max(0.1, self.v * (dist / self.slow_down_dist))
    
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

        self.get_logger().debug(f"[Control] Dist: {dist:.2f}m, W: {w:.2f} rad/s, V: {v:.2f} m/s")

    def fix_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def stop_robot(self):
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        self.cmd_pub.publish(stop)
        self.get_logger().info("stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = FollowerPIDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
