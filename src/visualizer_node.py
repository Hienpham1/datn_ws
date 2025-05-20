import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray
import math

class PoseArrayVisualizer(Node):
    def __init__(self):
        super().__init__('pose_array_visualizer')

        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.pose_sub = self.create_subscription(PoseArray, '/planned_path', self.pose_callback, 10)

        self.get_logger().info('PoseArray Visualizer node started...')

    def pose_callback(self, msg):
        markers = MarkerArray()

        for idx, pose in enumerate(msg.poses):
            marker = Marker()
            marker.header.frame_id = msg.header.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "poses"
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.3  # mũi tên dài
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime.sec = 0  # không tự biến mất
            markers.markers.append(marker)

        self.marker_pub.publish(markers)
        self.get_logger().info(f'Visualizing {len(markers.markers)} poses...')

def main(args=None):
    rclpy.init(args=args)
    node = PoseArrayVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()