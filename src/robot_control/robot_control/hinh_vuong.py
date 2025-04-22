import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class PoseToVelocityNode(Node):

    def __init__(self):
        super().__init__('pose_to_velocity_node')

        # Parameters
        self.declare_parameter('goal_tolerance_position', 0.05)
        self.declare_parameter('goal_tolerance_angle', 0.1)  # radians
        self.declare_parameter('max_linear_speed', 0.2)
        self.declare_parameter('max_angular_speed', 0.5)

        self.goal_tolerance_position = self.get_parameter('goal_tolerance_position').value
        self.goal_tolerance_angle = self.get_parameter('goal_tolerance_angle').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value

        # Publisher (v, w)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber (x, y, theta)
        self.odom_sub = self.create_subscription(Odometry, 'odom_encoder', self.odom_callback, 10)

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Waypoints list: [x, y, theta] in rad
        self.waypoints = [
            [3.0, 0.0, 0.0],
            [4.0, 0.0, -math.pi/2],
            [3.0, -1.0, math.pi/2],
            [3.0, 0.0, math.pi/2],#xong gocs 1
            [3.0, 2.0, math.pi/2],
            [3.0, 3.0, 0],
            [22.0, 10.0, math.pi],
            [20.0, 10.0, math.pi],#xong gocs 2
            [0.0, 10.0, math.pi],
            [-2.0, 10.0, math.pi/2],
            [0.0, 12.0, -math.pi/2],
            [0.0, 10.0, -math.pi/2],#xong goc 3
            [0.0, 0.0, -math.pi/2],
        ]
        self.current_goal_index = 0
        self.reached = False

        # Current pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Convert quaternion to yaw (theta)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

    def timer_callback(self):
        if self.current_goal_index >= len(self.waypoints):
            self.get_logger().info("da hoan thanh cac diem")
            self.publish_velocity(0.0, 0.0)
            return

        goal_x, goal_y, goal_theta = self.waypoints[self.current_goal_index]
        dx = goal_x - self.x
        dy = goal_y - self.y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        heading_error = self.normalize_angle(angle_to_goal - self.theta)
        angle_error = self.normalize_angle(goal_theta - self.theta)

        if distance > self.goal_tolerance_position:
            # di thang
            linear = self.max_linear_speed * math.cos(heading_error)
            angular = self.max_angular_speed * heading_error
            self.reached = False
        elif abs(angle_error) > self.goal_tolerance_angle:
            # xoay tai cho
            linear = 0.0
            angular = self.max_angular_speed * angle_error
            self.reached = False
        else:
            # dat den diem dich
            linear = 0.0
            angular = 0.0
            if not self.reached:
                self.get_logger().info(f"da toi diem:{self.current_goal_index + 1}/{len(self.waypoints)}")
                self.reached = True
                self.current_goal_index += 1

        self.publish_velocity(linear, angular)

    def publish_velocity(self, v, w):
        msg = Twist()
        msg.linear.x = max(min(v, self.max_linear_speed), -self.max_linear_speed)
        msg.angular.z = max(min(w, self.max_angular_speed), -self.max_angular_speed)
        self.cmd_pub.publish(msg)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = PoseToVelocityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

