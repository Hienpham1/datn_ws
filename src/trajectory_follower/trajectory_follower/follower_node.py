import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import math

class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower_node')

        # Declare & get initial parameters
        self.declare_parameter('Kx', 5.0)
        self.declare_parameter('Ktheta', 5.0)
        self.Kx = self.get_parameter('Kx').value
        self.Ktheta = self.get_parameter('Ktheta').value

        # Subscriptions
        self.pose_sub = self.create_subscription(PoseArray, '/planned_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom_encoder', self.odom_callback, 10)
        self.k_sub = self.create_subscription(Float64MultiArray, '/k_values', self.k_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Internal state
        self.path = []
        self.current_target_index = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Trajectory Follower node started.')

    def path_callback(self, msg):
        self.path = msg.poses
        self.current_target_index = 0
        self.get_logger().info(f'Received new path with {len(self.path)} waypoints.')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_theta = self.quaternion_to_yaw(q)

    def k_callback(self, msg):
        if len(msg.data) >= 2:
            self.Kx = msg.data[0]
            self.Ktheta = msg.data[1]
            self.get_logger().info(f'Updated K values: Kx = {self.Kx:.2f}, Ktheta = {self.Ktheta:.2f}')

    def control_loop(self):
        if not self.path or self.current_target_index >= len(self.path):
            return

        target_pose = self.path[self.current_target_index]
        dx = target_pose.position.x - self.robot_x
        dy = target_pose.position.y - self.robot_y

        rho = math.sqrt(dx**2 + dy**2)
        theta_target = math.atan2(dy, dx)
        error_theta = self.normalize_angle(theta_target - self.robot_theta)

        v = self.Kx * rho
        w = self.Ktheta * error_theta

        # Move to next waypoint if close enough
        if rho < 0.1:
            if self.current_target_index < len(self.path) - 1:
                self.current_target_index += 1
                self.get_logger().info(f'Switching to next waypoint: {self.current_target_index}')
            else:
                v = 0.0
                w = 0.0
                self.get_logger().info('Reached final waypoint.')

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_pub.publish(twist)

    def quaternion_to_yaw(self, q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
