import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from gpiozero import DigitalInputDevice
from gpiozero import RotaryEncoder
import time
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class GPIOZeroEncoderNode(Node):
    def __init__(self):
        super().__init__('gpiozero_encoder_node')

        # Encoder A/B GPIO pins
        self.encoder_Left = RotaryEncoder(17, 27, max_steps=0)
        self.encoder_Right = RotaryEncoder(22, 23, max_steps=0)

        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0

        # Robot parameters
        self.r = 0.05      # wheel radius (m)
        self.L = 0.32      # wheel base (m)
        self.ppr = 1000.0  # pulses per revolution

        # Pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pre_theta = 0.0
        self.pre_x = 0.0
        self.pre_y = 0.0

        self.odom_pub = self.create_publisher(Odometry, '/odom_encoder', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.prev_time = time.time()
        self.timer = self.create_timer(0.05, self.update_odom)  # 20 Hz

    def update_odom(self):
        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now

        left_ticks = self.encoder_Left.steps
        right_ticks = -(self.encoder_Right.steps)

        delta_left = left_ticks - self.last_left_ticks
        delta_right = right_ticks - self.last_right_ticks
        
        dl = delta_left * (2 * math.pi * self.r / self.ppr)
        dr = delta_right * (2 * math.pi * self.r / self.ppr)
        self.get_logger().info(f"en1= {delta_right}, en2= {delta_right}")
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks

        self.get_logger().info(f"Dl= {dl}, Dr= {dr}")
        Davg = (dl + dr) / 2 
        Delta_theta = (dr - dl) / self.L #check lại chỗ này.
        self.get_logger().info(f"Davg={Davg}, Delta_theta={Delta_theta}") 
        Delta_x = Davg * math.cos(self.pre_theta + Delta_theta / 2)
        Delta_y = Davg * math.sin(self.pre_theta + Delta_theta / 2)
        
        self.theta = self.pre_theta + Delta_theta
        self.x = self.pre_x + Delta_x
        self.y = self.pre_y + Delta_y
        self.get_logger().info(f"x={self.x}, y={self.y}, theta={self.theta}, pre_theta={self.pre_theta}") 
        v = Davg / dt
        w = Delta_theta / dt
        self.get_logger().info(f"v={v}, w={w}") 
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = self.yaw_to_quat(self.theta)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        # Create TF transform
        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        q = self.yaw_to_quat(self.theta)
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

    def yaw_to_quat(self, yaw):
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0)
        )

def main(args=None):
    rclpy.init(args=args)
    node = GPIOZeroEncoderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
