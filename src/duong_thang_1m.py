import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import sqrt

class DriveStraight1Meter(Node):
    def __init__(self):
        super().__init__('drive_straight_1m')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.initial_pose = None
        self.initial_yaw = None
        self.distance = 0.0
        self.speed = 0.2
        self.Kp_angle = 1.5
        self.max_angular = 0.3

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        # Convert quaternion to yaw (theta)
        import tf_transformations
        (_, _, yaw) = tf_transformations.euler_from_quaternion(
            [ori.x, ori.y, ori.z, ori.w]
        )

        if self.initial_pose is None:
            self.initial_pose = pos
            self.initial_yaw = yaw
            return

        dx = pos.x - self.initial_pose.x
        dy = pos.y - self.initial_pose.y
        self.distance = sqrt(dx**2 + dy**2)

        # Giữ robot đi thẳng bằng cách giữ yaw ban đầu
        yaw_error = self.initial_yaw - yaw
        twist = Twist()

        if self.distance < 1.0:
            twist.linear.x = self.speed
            twist.angular.z = max(-self.max_angular, min(self.Kp_angle * yaw_error, self.max_angular))
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('Robot đã đi đúng 1 mét!')
            self.destroy_node()  # Dừng node

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = DriveStraight1Meter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
