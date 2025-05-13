import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32MultiArray
import math
import time
from nav_msgs.msg import Odometry

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output


class InverseKineticNode(Node):
    def __init__(self):
        super().__init__('inverse_kinetic')

        self.pid_left = PID(Kp=0.0, Ki=0.0, Kd=0.0)
        self.pid_right = PID(Kp=0.0, Ki=0.0, Kd=0.0)

        # Declare and get parameters
        self.declare_parameter('topic_velocities', 'topic_velocities')
        self.declare_parameter('cmd_vel', 'cmd_vel')
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheels_distance', 0.305)
        self.declare_parameter('t_acceleration', 1.0)
        self.declare_parameter('t_decreasing', 0.5)
        self.declare_parameter('v_max_motor', 310)
        self.declare_parameter('pwm_max', 255)

        self.topic_velocities = self.get_parameter('topic_velocities').get_parameter_value().string_value
        self.cmd_vel = self.get_parameter('cmd_vel').get_parameter_value().string_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheels_distance = self.get_parameter('wheels_distance').get_parameter_value().double_value
        self.t_acceleration = self.get_parameter('t_acceleration').get_parameter_value().double_value
        self.t_decreasing = self.get_parameter('t_decreasing').get_parameter_value().double_value
        self.v_max_motor = self.get_parameter('v_max_motor').get_parameter_value().integer_value
        self.pwm_max = self.get_parameter('pwm_max').get_parameter_value().integer_value

        # Initialize variables
        self.true_linear_vel = 0.0
        self.realtime_t = 0.0
        self.up_cnt = 0
        self.down_cnt = 0
        self.left_pwm = 0
        self.right_wm = 0   
        self.current_left = 0.0
        self.current_right = 0.0
        self.desired_left = 0.0
        self.desired_right = 0.0
        self.prev_time = time.time()

        # Publisher and Subscriber
        self.pub_v = self.create_publisher(Int16MultiArray, self.topic_velocities, 10)
        self.pub_desired_velo = self.create_publisher(Float32MultiArray, 'desired_velo', 10)
        self.pub_current_velo = self.create_publisher(Float32MultiArray, 'current_velo', 10)

        self.sub = self.create_subscription(Twist, self.cmd_vel, self.velocities_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom_encoder', self.odom_callback, 10)
        self.sub_pid_left = self.create_subscription(Float32MultiArray, '/pid_params_left', self.update_pid_left, 10)
        self.sub_pid_right = self.create_subscription(Float32MultiArray, '/pid_params_right', self.update_pid_right, 10)


    def update_pid_left(self, msg):
        kp, ki, kd = msg.data
        self.pid_linear.Kp = kp
        self.pid_linear.Ki = ki
        self.pid_linear.Kd = kd
        self.get_logger().info(f"Updated left PID: Kp={kp}, Ki={ki}, Kd={kd}")

    def update_pid_right(self, msg):
        kp, ki, kd = msg.data
        self.pid_angular.Kp = kp
        self.pid_angular.Ki = ki
        self.pid_angular.Kd = kd
        self.get_logger().info(f"Updated right PID: Kp={kp}, Ki={ki}, Kd={kd}")


    def odom_callback(self, msg):
        current_linear_vel = msg.twist.twist.linear.x
        current_angular_vel = msg.twist.twist.angular.z
        self.current_left = (1.0 / (2.0 * self.wheel_radius)) * (2.0 * current_linear_vel - self.wheels_distance * current_angular_vel)
        self.current_right = (1.0 / (2.0 * self.wheel_radius)) * (2.0 * current_linear_vel + self.wheels_distance * current_angular_vel)

        # Publish current velocities
        current_msg = Float32MultiArray()
        current_msg.data = [self.current_left, self.current_right]
        self.pub_current_velo.publish(current_msg)

    def IK(self, linear_vel, angular_vel):
        # Tính toán tốc độ mong muốn cho từng bánh (rad/s)
        self.desired_left = (1.0 / (2.0 * self.wheel_radius)) * (2.0 * linear_vel - self.wheels_distance * angular_vel)
        self.desired_right = (1.0 / (2.0 * self.wheel_radius)) * (2.0 * linear_vel + self.wheels_distance * angular_vel)

        # Publish desired velocities
        desired_msg = Float32MultiArray()
        desired_msg.data = [self.desired_left, self.desired_right]
        self.pub_desired_velo.publish(desired_msg)

        self.get_logger().info(f"Desired velocities - left: {self.desired_left:.2f} rad/s, right: {self.desired_right:.2f} rad/s")
        
    def velocity_handling(self):
        msg = Int16MultiArray()
        msg.data = [-self.left_pwm, self.right_pwm]
        self.pub_v.publish(msg)
    
    def control(self, desired_linear, desired_angular):
        dt = time.time() - self.prev_time
        self.prev_time = time.time()

        #tinh toc do mong muon
        self.IK(desired_linear, desired_angular)

        #tinh sai so
        error_left = self.desired_left - self.current_left
        error_right = self.desired_right - self.current_right
        
        #compute gia tri voi pid
        pwm_left = self.pid_left.compute(error_left, dt)
        pwm_right = self.pid_right.compute(error_right, dt)

        # gioi han gia tri pwm
        corrected_left = max(minpwm_left, 255), -255)
        corrected_right = max(minpwm_right, 255), -255)

        # gui pwm
        self.left_pwm = corrected_left
        self.right_pwm = corrected_right  
        self.velocity_handling()
	self.get_logger().info(f"pwm corrected - left: {corrected_left}, right: {corrected_right}")

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
