import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, Quaternion
import math

def euler_to_quaternion(yaw):
    """ Chuyển yaw (rad) -> quaternion ROS """
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0)
    )

def apply_offset(x, y, theta, offset=0.245):
    """Chuyển từ pose của đầu vẽ ➜ pose của robot"""
    x_r = x - offset * math.cos(theta)
    y_r = y - offset * math.sin(theta)
    return x_r, y_r, theta

class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner_node')

        self.pose_pub = self.create_publisher(PoseArray, '/planned_path', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_path)

        self.get_logger().info('Trajectory Planner node started...')

    def publish_path(self):
        pose_array = PoseArray()
        pose_array.header.frame_id = 'map'
        pose_array.header.stamp = self.get_clock().now().to_msg()

        # quỹ đạo
        # waypoints = [
        #     (0.0, 0.0, 0.0),       # Start
        #     #(4.0, 0.0, 0.0),       # Line A->B
        #     (4.0, 0.0, math.pi/2), # Chuẩn bị rẽ trái
        #     #(4.0, 2.245, math.pi/2),
        #     (4.0, 4.0, math.pi),
        #     (0.0, 4.0, -math.pi/2), 
        #     (0.0, 0.0, 0.0),       # Về lại điểm xuất phát
        # ]

        # for x, y, yaw in waypoints:
        #     pose = Pose()
        #     pose.position.x = x
        #     pose.position.y = y
        #     pose.position.z = 0.0
        #     pose.orientation = euler_to_quaternion(yaw)
        #     pose_array.poses.append(pose)

        # self.pose_pub.publish(pose_array)
        # self.get_logger().info('Published planned path with %d poses.' % len(pose_array.poses))

        #hinh vuong
        # square_points = [
        #     (0.0, 0.0, 0.0),
        #     (4.0, 0.0, 0.0),
        #     (4.0, 4.0, math.pi/2),
        #     (0.0, 4.0, math.pi),
        #     (0.0, 0.0, -math.pi/2)
        # ]
        # for x, y, yaw in square_points:
        #     x_r, y_r, yaw_r = apply_offset(x, y, yaw)
        #     pose = Pose()
        #     pose.position.x = x_r
        #     pose.position.y = y_r
        #     pose.orientation = euler_to_quaternion(yaw_r)
        #     pose_array.poses.append(pose)
        
        # #di chuyen den B
        # x_b, y_b, theta_b = 2.0, 0.0, math.pi/2  # hướng lên
        # x_r, y_r, yaw_r = apply_offset(x_b, y_b, theta_b)
        # pose = Pose()
        # pose.position.x = x_r
        # pose.position.y = y_r
        # pose.orientation = euler_to_quaternion(yaw_r)
        # pose_array.poses.append(pose)

        #hinh tron
        center_x, center_y = 2.0, 2.0
        radius = 2.0
        num_points = 100
        for i in range(num_points + 1):
            angle = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            theta = angle + math.pi/2  # hướng tiếp tuyến
            x_r, y_r, yaw_r = apply_offset(x, y, theta)
            pose = Pose()
            pose.position.x = x_r
            pose.position.y = y_r
            pose.orientation = euler_to_quaternion(yaw_r)
            pose_array.poses.append(pose)
        
        self.pose_pub.publish(pose_array)
        self.get_logger().info(f'Published full path: {len(pose_array.poses)} poses.')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()