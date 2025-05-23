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
    x_r = x + offset * math.cos(theta)
    y_r = y + offset * math.sin(theta)
    return x_r, y_r, theta

class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('duong_thang')

        self.pose_pub = self.create_publisher(PoseArray, '/planned_path', 10)
        self.spray_pub = self.create_publisher(Int32, '/son', 10)

        self.timer = self.create_timer(1.0, self.publish_path)

        self.get_logger().info('Trajectory Planner (A→B) started...')

    def publish_path(self):
        pose_array = PoseArray()
        pose_array.header.frame_id = 'odom'
        pose_array.header.stamp = self.get_clock().now().to_msg()

        waypoints = []

        # Điểm bắt đầu (tắt sơn)
        start_x, start_y, start_theta = apply_offset(0.0, 0.0, 0.0)
        waypoints.append((start_x, start_y, start_theta))
        self.spray_pub.publish(Int32(data=0))

        # Bật sơn và đi từ A → B
        self.spray_pub.publish(Int32(data=1))
        
        waypoints = [
            (1.5, 0.0, 0.0),   # go straight to x=1
            # (2.0, 1.0, 0.0),   # turn right/up
            # (3.0, 0.0, 0.0),   # turn back down
        ]
        self.spray_pub.publish(Int32(data=0))  # Tắt sơn khi kết thúc

        for x, y, yaw in waypoints:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.orientation = euler_to_quaternion(yaw)
            pose_array.poses.append(pose)
	
        #waypoints = []
    	#Tinh so diem tren duong tron
        #num_points = 36  # Ví dụ: 36 điểm cách nhau 10 độ
        #for i in range(num_points):
        #    theta = (i / num_points) * 2 * math.pi  # Góc quay từ 0 đến 2pi
        #    x = 0 + 2 * math.cos(theta)  # Tâm (0,2), bán kính 2
        #    y = 2 + 2 * math.sin(theta)
        #    yaw = theta  # Hướng quay theo đường tròn
        #    pose = Pose()
        #    pose.position.x = x
        #    pose.position.y = y
        #    pose.orientation = euler_to_quaternion(yaw)
        #    pose_array.poses.append(pose)

        self.pose_pub.publish(pose_array)
        self.get_logger().info(f"Đã gửi đường đi A → B với {len(pose_array.poses)} điểm.")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
