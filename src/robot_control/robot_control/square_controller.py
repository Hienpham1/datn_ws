import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('square_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Khai báo các waypoint và đoạn bo góc
        self.waypoints = [
            # A -> B
            {"type": "line", "x": 20.0, "y": 0.0, "paint": True},
            # Bo góc B (phải -> trái)
            {"type": "line", "x": 22.0, "y": 0.0, "paint": False},
            {"type": "arc", "radius": 2.0, "angle_deg": 90, "direction": "left", "paint": False},
            {"type": "line", "x": 20.0, "y": 0.0, "paint": False},
            # B -> E
            {"type": "line", "x": 20.0, "y": 10.0, "paint": True},

            # Bo góc E
            {"type": "line", "x": 20.0, "y": 12.0, "paint": False},
            {"type": "arc", "radius": 2.0, "angle_deg": 90, "direction": "left", "paint": False},
            {"type": "line", "x": 20.0, "y": 10.0, "paint": False},
            # E -> góc dưới trái
            {"type": "line", "x": 0.0, "y": 10.0, "paint": True},

            # Bo góc trái
            {"type": "line", "x": -2.0, "y": 10.0, "paint": False},
            {"type": "arc", "radius": 2.0, "angle_deg": 90, "direction": "left", "paint": False},
            {"type": "line", "x": 0.0, "y": 10.0, "paint": False},
            # Trở về A
            {"type": "line", "x": 0.0, "y": 0.0, "paint": True},
        ]

        self.linear_speed = 0.2  # m/s
        self.angular_speed = math.radians(30)  # rad/s

        self.run()

    def run(self):
        for wp in self.waypoints:
            if wp["type"] == "line":
                self.go_straight(wp["x"], wp["y"], wp["paint"])
            elif wp["type"] == "arc":
                self.turn_arc(wp["radius"], wp["angle_deg"], wp["direction"], wp["paint"])
        self.stop()

    def go_straight(self, x, y, paint):
        # 👉 Thay bằng điều hướng tự động sau này
        distance = math.sqrt((x)**2 + (y)**2)
        duration = distance / self.linear_speed

        twist = Twist()
        twist.linear.x = self.linear_speed

        # 👉 Bật bơm nếu cần
        if paint:
            # self.paint_pub.publish(Bool(data=True))
            pass

        self.cmd_vel_pub.publish(twist)
        time.sleep(duration)

        self.stop()
        # 👉 Tắt bơm nếu cần
        if paint:
            # self.paint_pub.publish(Bool(data=False))
            pass

    def turn_arc(self, radius, angle_deg, direction, paint):
        angular_rad = math.radians(angle_deg)
        duration = angular_rad / self.angular_speed

        twist = Twist()
        twist.angular.z = self.angular_speed if direction == "left" else -self.angular_speed
        twist.linear.x = radius * abs(twist.angular.z)

        self.cmd_vel_pub.publish(twist)
        time.sleep(duration)

        self.stop()

    def stop(self):
        self.cmd_vel_pub.publish(Twist())
        time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

