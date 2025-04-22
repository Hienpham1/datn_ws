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
        self.declare_parameter('kp_pos', 2.0)
        self.declare_parameter('kd_pos', 0.0)
        self.declare_parameter('kp_theta', 2.0)
        self.declare_parameter('kd_theta', 0.1)

        self.kp_pos = self.get_parameter('kp_pos').value
        self.kd_pos = self.get_parameter('kd_pos').value
        self.kp_theta = self.get_parameter('kp_theta').value
        self.kd_theta = self.get_parameter('kd_theta').value

        self.declare_parameter('waypointfile', 'waypoints.yaml')
        waypointfile = self.get_parameter('waypointfile').value
        self.load_waypoints(waypointfile)
        self.current_wp_index = 0

        self.odom_sub = self.create_subscription(Odometry, '/odom_encoder', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_pd', 10)

        self.error_pos_pub = self.create_publisher(Float32, '/error_pos', 10)
        self.error_theta_pub = self.create_publisher(Float32, '/error_theta', 10)

        self.goal_pose = None
        self.prev_pos_error = 0.0
        self.prev_theta_error = 0.0
        self.dt = 0.1  # assume 10Hz for now
        self.timer = self.create_timer(self.dt, self.control_loop)

    def load_waypoints(self, file_path):
        script_dir = os.path.dirname(os.path.realpath(__file__))
        full_path = os.path.join(script_dir, file_path)
        with open(full_path, 'r') as f:
            data = yaml.safe_load(f)
        self.waypoints = data['waypoints']
        

    def goal_callback(self, msg):
        self.goal_pose = msg.pose

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def control_loop(self):
        if self.goal_pose is None or self.current_wp_index >= len(self.waypoints):
            return

        x = self.current_pose.position.x
        y = self.current_pose.position.y

        wp=self.waypoints[self.current_wp_index]
        x_goal = wp['x']
        y_goal = wp['y']

        # Get current yaw
        q = self.current_pose.orientation
        yaw = self.get_yaw_from_quaternion(q)

        # Compute angle to goal
        dx = x_goal - x
        dy = y_goal - y
        target_angle = math.atan2(dy, dx)
        error_theta = self.normalize_angle(target_angle - yaw)
        error_pos = math.sqrt(dx**2 + dy**2)

        self.error_pos_pub.publish(Float32(data=error_pos))
        self.error_theta_pub.publish(Float32(data=error_theta))

        d_error_pos = (error_pos - self.prev_pos_error) / self.dt
        d_error_theta = (error_theta - self.prev_theta_error) / self.dt

        v = self.kp_pos * error_pos + self.kd_pos * d_error_pos
        w = self.kp_theta * error_theta + self.kd_theta * d_error_theta

        twist = Twist()
        twist.linear.x = v if error_pos > 0.05 else 0.0
        twist.angular.z = w if abs(error_theta) > 0.05 else 0.0
        self.cmd_pub.publish(twist)

        if error_pos < 0.1 and abs(error_theta) < 0.1:
            self.get_logger().info(f"reach {self.current_wp_index +1}")
            #self.prev_pos_error = error_pos
            #self.prev_theta_error = error_theta
            self.current_wp_index +=1

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
