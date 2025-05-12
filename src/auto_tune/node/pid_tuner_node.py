import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Float32MultiArray

from Simulations import Simulator
from PIDController import PIDController
from tuner import PIDTuner  # Giả sử bạn đã lưu class PIDTuner trong tuner.py

class PIDTuningNode(Node):

    def __init__(self):
        super().__init__('pid_tuning_node')

        # Subscriptions
        self.create_subscription(Odometry, '/odom_encoder', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/path_planned', self.path_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Float32MultiArray, '/topic_velocities', self.pwm_callback, 10)

        # Data holders
        self.odom = None
        self.path = None
        self.cmd_vel = None
        self.pwm = None

        # Timer for tuning (can be triggered manually if you prefer)
        self.create_timer(2.0, self.run_tuner)

    def odom_callback(self, msg):
        self.odom = msg

    def path_callback(self, msg):
        self.path = msg

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg

    def pwm_callback(self, msg):
        self.pwm = msg

    def run_tuner(self):
        if None in (self.odom, self.path, self.cmd_vel, self.pwm):
            self.get_logger().info('Waiting for all topics...')
            return

        # Tạo simulator từ dữ liệu hiện tại
        sim = Simulator(...)  # Cần bạn cài đặt cụ thể
        setpoint = self.cmd_vel.linear.x  # hoặc setpoint khác phù hợp
        t0, t1, dt = 0.0, 5.0, 0.1

        tuner = PIDTuner(sim, setpoint, t0, t1, dt)
        for _ in range(30):  # epochs để tối ưu
            tuner.epoch()

        kp, ki, kd = tuner.get_vals()
        self.get_logger().info(f"Optimal PID: Kp={kp:.3f}, Ki={ki:.3f}, Kd={kd:.3f}")
