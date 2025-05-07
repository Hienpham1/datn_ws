import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math
from std_msgs.msg import Float32
import os
import yaml

class PDPoseController(Node):

    def __init__(self):
        super().__init__('pd_pose_controller')
        
        # Parameters
        self.declare_parameters(namespace='',
            parameters=[
                ('kp_pos', 2.0),
                ('kd_pos', 2.0),
                ('kp_theta', 0.5),
                ('kd_theta', 0.5),
                ('waypointfile', 'waypoints.yaml'),
                ('pos_tolerance', 0.1),
                ('angle_tolerance', 0.1),
                ('max_linear_vel', 0.5),
                ('max_angular_vel', 1.0)
            ])

        self.kp_pos = self.get_parameter('kp_pos').value
        self.kd_pos = self.get_parameter('kd_pos').value
        self.kp_theta = self.get_parameter('kp_theta').value
        self.kd_theta = self.get_parameter('kd_theta').value
        self.pos_tol = self.get_parameter('pos_tolerance').value
        self.angle_tol = self.get_parameter('angle_tolerance').value
        self.max_v = self.get_parameter('max_linear_vel').value
        self.max_w = self.get_parameter('max_angular_vel').value

        # Load waypoints
        self.load_waypoints(self.get_parameter('waypointfile').value)
        self.current_wp_index = 0
        self.waypoints_completed = False

        # Subscribers and Publishers
        self.odom_sub = self.create_subscription(Odometry, '/odom_encoder', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # For visualization and debugging
        self.error_pos_pub = self.create_publisher(Float32, '/error_pos', 10)
        self.error_theta_pub = self.create_publisher(Float32, '/error_theta', 10)
        self.current_pose_pub = self.create_publisher(PoseStamped, '/current_pose_viz', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose_viz', 10)

        # Control variables
        self.goal_pose = None
        self.current_pose = None
        self.prev_pos_error = 0.0
        self.prev_theta_error = 0.0
        self.dt = 0.1  # 10Hz control loop
        self.timer = self.create_timer(self.dt, self.control_loop)

    def load_waypoints(self, file_path):
        script_dir = os.path.dirname(os.path.realpath(__file__))
        full_path = os.path.join(script_dir, file_path)
        
        try:
            with open(full_path, 'r') as f:
                data = yaml.safe_load(f)
            self.waypoints = data['waypoints']
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            self.waypoints = []

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.publish_goal_pose_viz()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.publish_current_pose_viz()
    
    def publish_current_pose_viz(self):
        if self.current_pose:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.pose = self.current_pose
            self.current_pose_pub.publish(msg)

    def publish_goal_pose_viz(self):
        """Publish goal pose for visualization in Rviz"""
        if self.goal_pose:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.pose = self.goal_pose
            self.goal_pose_pub.publish(msg)

    def control_loop(self):
        if not self.waypoints or self.waypoints_completed:
            self.publish_zero_velocity()
            return

        if not self.current_pose:
            return

        wp=self.waypoints[self.current_wp_index]
        x_goal = wp['x']
        y_goal = wp['y']

        x = self.current_pose.position.x
        y = self.current_pose.position.y
        yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)

        # Compute angle to goal
        dx = x_goal - x
        dy = y_goal - y
        error_pos = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        error_theta = self.normalize_angle(target_angle - yaw)

        self.error_pos_pub.publish(Float32(data=error_pos))
        self.error_theta_pub.publish(Float32(data=error_theta))

        d_error_pos = (error_pos - self.prev_pos_error) / self.dt
        d_error_theta = (error_theta - self.prev_theta_error) / self.dt

        #PD control
        v = self.kp_pos * error_pos + self.kd_pos * d_error_pos
        w = self.kp_theta * error_theta + self.kd_theta * d_error_theta

        # Apply velocity limits
        v = max(min(v, self.max_v), -self.max_v)
        w = max(min(w, self.max_w), -self.max_w)

        twist = Twist()
        if error_pos > self.pos_tol:
            twist.linear.x = v
        if abs(error_theta) > self.angle_tol:
            twist.angular.z = w
        self.cmd_pub.publish(twist)

        # Update previous errors
        self.prev_pos_error = error_pos
        self.prev_theta_error = error_theta

        # Check if waypoint reached
        if error_pos < self.pos_tol and abs(error_theta) < self.angle_tol:
            self.get_logger().info(f"Reached waypoint {self.current_wp_index + 1}/{len(self.waypoints)}")
            self.current_wp_index += 1
            
            if self.current_wp_index >= len(self.waypoints):
                self.waypoints_completed = True
                self.get_logger().info("All waypoints completed!")
                self.publish_zero_velocity()
                
    def publish_zero_velocity(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def get_yaw_from_quaternion(self, q):
        # Convert quaternion to yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = PDPoseController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
