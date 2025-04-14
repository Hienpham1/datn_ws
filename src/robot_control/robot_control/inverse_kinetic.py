import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
import math
import time


class InverseKineticNode(Node):
    def __init__(self):
        super().__init__('inverse_kinetic')

        # Declare and get parameters
        self.declare_parameter('topic_velocities', 'topic_velocities')
        self.declare_parameter('cmd_vel', 'cmd_vel')
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheels_distance', 0.32)
        self.declare_parameter('steering_factor', 71.2) #dieu chinh cho nay 71.2 
        self.declare_parameter('t_acceleration', 1.0)
        self.declare_parameter('t_decreasing', 0.5)
        self.declare_parameter('v_max_motor', 310)
        self.declare_parameter('pwm_max', 255)

        self.topic_velocities = self.get_parameter('topic_velocities').get_parameter_value().string_value
        self.cmd_vel = self.get_parameter('cmd_vel').get_parameter_value().string_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheels_distance = self.get_parameter('wheels_distance').get_parameter_value().double_value
        self.steering_factor = self.get_parameter('steering_factor').get_parameter_value().double_value
        self.t_acceleration = self.get_parameter('t_acceleration').get_parameter_value().double_value
        self.t_decreasing = self.get_parameter('t_decreasing').get_parameter_value().double_value
        self.v_max_motor = self.get_parameter('v_max_motor').get_parameter_value().integer_value
        self.pwm_max = self.get_parameter('pwm_max').get_parameter_value().integer_value

        # Initialize variables
        self.true_linear_vel = 0.0
        self.realtime_t = 0.0
        self.up_cnt = 0
        self.down_cnt = 0
        self.left_wheel_velocity = 0
        self.right_wheel_velocity = 0
        self.prev_time = time.time()

        # Publisher and Subscriber
        self.pub_v = self.create_publisher(Int16MultiArray, self.topic_velocities, 10)
        self.sub = self.create_subscription(Twist, self.cmd_vel, self.velocities_callback, 10)

    def convert_pwm(self, wheel_vel):
        #pwm = wheel_vel * self.steering_factor
        pwm = (wheel_vel / self.v_max_motor) * self.pwm_max
        return int(pwm)

    def IK(self, linear_vel, angular_vel):
        omega_left = (1.0 / (2.0 * self.wheel_radius)) * (2.0 * linear_vel - self.wheels_distance * angular_vel)
        omega_right = (1.0 / (2.0 * self.wheel_radius)) * (2.0 * linear_vel + self.wheels_distance * angular_vel)
        self.left_wheel_velocity = self.convert_pwm(omega_left * (60 / (2 * math.pi)))
        self.right_wheel_velocity = self.convert_pwm(omega_right * (60 / (2 * math.pi)))
        self.get_logger().info(f"pub velo: left={-self.left_wheel_velocity}, right={self.right_wheel_velocity}")
        self.get_logger().info(f"omega_left={omega_left}, omega_right={omega_right}")
    def velocity_handling(self):
        msg = Int16MultiArray()
        msg.data = [-self.left_wheel_velocity, self.right_wheel_velocity]
        self.pub_v.publish(msg)

    def decreasing_acceleration(self, linear_vel):
        current_time = time.time()
        dt = current_time - self.prev_time

        if self.true_linear_vel != linear_vel and linear_vel != 0:
            self.down_cnt = 0
            if self.up_cnt < 2:
                self.realtime_t = 0
                self.acc = linear_vel / self.t_acceleration
                self.up_cnt += 1
            if self.up_cnt > 1:
                self.realtime_t += dt
            if self.realtime_t > self.t_acceleration:
                self.realtime_t = self.t_acceleration

            self.true_linear_vel = self.acc * self.realtime_t
            self.prev_time = current_time

        elif self.true_linear_vel != linear_vel and linear_vel == 0:
            self.up_cnt = 0
            if self.down_cnt < 2:
                self.realtime_t = 0
                self.speed_decreasing_const = self.true_linear_vel / self.t_decreasing
                self.down_cnt += 1
            if self.down_cnt > 1:
                self.realtime_t += dt
            if self.realtime_t > self.t_decreasing:
                self.realtime_t = self.t_decreasing
                self.true_linear_vel = 0
            else:
                self.true_linear_vel = self.speed_decreasing_const * self.t_decreasing - (self.speed_decreasing_const * self.realtime_t)
            self.prev_time = current_time

        else:
            self.realtime_t = 0
            self.up_cnt = 0
            self.down_cnt = 0

    def control(self, linear_vel, angular_vel):
        self.decreasing_acceleration(linear_vel)
        self.IK(self.true_linear_vel, angular_vel)
        self.velocity_handling()

    def velocities_callback(self, msg):
        self.get_logger().info(f"Received velocities: linear={msg.linear.x}, angular={msg.angular.z}")
        self.control(msg.linear.x, msg.angular.z)


def main(args=None):
    rclpy.init(args=args)
    node = InverseKineticNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()