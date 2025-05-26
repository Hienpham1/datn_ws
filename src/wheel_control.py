import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int16MultiArray


class SimpleKalmanFilter:
    def __init__(self, P=1.0, Q=0.01, R=0.1):
        self.P = P
        self.Q = Q
        self.R = R
        self.x = 0.0

    def update(self, y):
        self.P += self.Q
        K = self.P / (self.P + self.R)
        self.x += K * (y - self.x)
        self.P = (1 - K) * self.P
        return self.x

class LowPassFilter:
    def __init__(self, alpha=0.2):
        self.alpha = alpha 
        self.filtered_value = 0.0

    def filter(self, raw_value):
        self.filtered_value = self.alpha * raw_value + (1 - self.alpha) * self.filtered_value
        return self.filtered_value
    
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        if (self.Ki != 0):
            limit = 1*255/self.Ki
            self.integral += error * dt
            self.integral = max(-(limit), min(self.integral,limit))
        else:
            self.integral = 0  
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

# ========== Node Control speed of Wheels ========= 
class wheels_control(Node):
    def __init__(self):
        super().__init__('wheels_control')
        self.pid_left = PID(Kp= 750.0, Ki=1000.0, Kd=30.0)
        self.pid_right = PID(Kp=750.0, Ki=1000.0, Kd=30.0)

        # Tạo các bộ lọc Kalman cho từng bánh
        self.kf_left = SimpleKalmanFilter(P=1.0, Q=0.01, R=0.1)
        self.kf_right = SimpleKalmanFilter(P=1.0, Q=0.01, R=0.1)

        # Hoặc nếu dùng Low-pass thay vì Kalman:
        self.lpf_left = LowPassFilter(alpha=0.2)
        self.lpf_right = LowPassFilter(alpha=0.2)

        # Declare and get parameters
        self.declare_parameter('topic_velocities', 'topic_velocities')
        self.declare_parameter('cmd_vel', 'cmd_vel')
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheels_distance', 0.305)
        self.declare_parameter('pwm_max', 255)

        self.topic_velocities = self.get_parameter('topic_velocities').get_parameter_value().string_value
        self.cmd_vel = self.get_parameter('cmd_vel').get_parameter_value().string_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheels_distance = self.get_parameter('wheels_distance').get_parameter_value().double_value
        self.pwm_max = self.get_parameter('pwm_max').get_parameter_value().integer_value

        # Initialize variables
        self.left_pwm = 0
        self.right_pwm = 0   
        self.PV_left = 0.0
        self.PV_right = 0.0
        self.SP_left = 0.0
        self.SP_right = 0.0
        self.dt = 0.05
        self.received_vel = False
        self.last_SP_left = 0.0
        self.last_SP_right = 0.0

        # Publisher and Subscriber
        self.pub_v = self.create_publisher(Int16MultiArray, self.topic_velocities, 10) # PWM
        self.pub_SP_velo = self.create_publisher(Float32MultiArray, 'SP_vel', 10)
        self.pub_PV_velo = self.create_publisher(Float32MultiArray, 'PV_vel', 10)

        self.sub = self.create_subscription(Twist, self.cmd_vel, self.SP_velo_callback, 10)
        self.sub_odom = self.create_subscription(Float32MultiArray, '/vel_wheels', self.current_velo_callback, 10)
        # self.sub_pid_left = self.create_subscription(Float32MultiArray, '/pid_params_left', self.update_pid_left, 10)
        # self.sub_pid_right = self.create_subscription(Float32MultiArray, '/pid_params_right', self.update_pid_right, 10)
        self.sub_gains_pid = self.create_subscription(Float32MultiArray, '/pid_params', self.update_pid, 10)
        
        self.timer = self.create_timer(self.dt, self.PID_control)

    # def update_pid_left(self, msg):
    #     self.pid_left.reset()
    #     kp, ki, kd = msg.data
    #     self.pid_left.Kp = kp
    #     self.pid_left.Ki = ki
    #     self.pid_left.Kd = kd

    # def update_pid_right(self, msg):
    #     self.pid_right.reset()
    #     kp, ki, kd = msg.data
    #     self.pid_right.Kp = kp
    #     self.pid_right.Ki = ki
    #     self.pid_right.Kd = kd

    def update_pid(self, msg):
        self.pid_left.reset()
        self.pid_right.reset()
        kp_l, ki_l, kd_l, kp_r, ki_r, kd_r = msg.data
        self.pid_left.Kp = kp_l
        self.pid_left.Ki = ki_l
        self.pid_left.Kd = kd_l
        self.pid_right.Kp = kp_r
        self.pid_right.Ki = ki_r
        self.pid_right.Kd = kd_r

        
    def current_velo_callback(self, msg):
        self.received_vel = True
        current_left, current_right = msg.data  # m/s
        self.get_logger().info(f"current l={current_left}, r={current_right}")

        #  ---APLY FILTER ---
        # filtered_left = self.kf_left.update(current_left)     # Kalman filter
        # filtered_right = self.kf_right.update(current_right) 

        filtered_left = self.lpf_left.filter(current_left)     # Low-pass filter
        filtered_right = self.lpf_right.filter(current_right)

        # data filtered
        self.PV_left = filtered_left
        self.PV_right = filtered_right

        #Publish filtered_velocities (PV)
        PV_msg = Float32MultiArray()
        PV_msg.data = [self.PV_left, self.PV_right]
        self.pub_PV_velo.publish(PV_msg)

    def SP_velo_callback(self, msg):
        SP_linear_vel = msg.linear.x
        SP_angular_vel = msg.angular.z
        self.get_logger().info(f"desird v={SP_linear_vel}, w={SP_angular_vel}")
        # Tính toán tốc độ mong muốn cho từng bánh (rad/s)
        self.SP_left = (2.0 * SP_linear_vel - self.wheels_distance * SP_angular_vel) / 2  # m/s
        self.SP_right =(2.0 * SP_linear_vel + self.wheels_distance * SP_angular_vel) / 2

        if abs(self.SP_left - self.last_SP_left) > 0.1:
            self.pid_left.reset()
            self.last_SP_left = self.SP_left
        if abs(self.SP_right - self.last_SP_right) > 0.1:
            self.pid_right.reset()
            self.last_SP_right = self.SP_right

        

        # self.get_logger().info(f"SP-left: {self.SP_left:.2f} rad/s, right: {self.SP_right:.2f} rad/s")
    
    def PID_control(self):
        if not self.received_vel:
            return 
        #tinh sai so
        error_left = self.SP_left - self.PV_left
        error_right = self.SP_right - self.PV_right
        self.get_logger().info(f"er_l = {error_left}, er_r= {error_right}")
        

        #compute gia tri voi pid
        pwm_left = self.pid_left.compute(error_left, self.dt)
        pwm_right = self.pid_right.compute(error_right, self.dt)

        # gioi han gia tri PWM
        self.left_pwm = max(min(pwm_left, self.pwm_max), -self.pwm_max)   # [-255:255]
        self.right_pwm = max(min(pwm_right, self.pwm_max), -self.pwm_max)

        # pub PWM
        msg = Int16MultiArray()
        msg.data = [int(-self.left_pwm), int(self.right_pwm)]
        self.pub_v.publish(msg) 

        # Publish desired velocities
        SP_msg = Float32MultiArray()
        SP_msg.data = [self.SP_left, self.SP_right]
        self.pub_SP_velo.publish(SP_msg)
        # self.get_logger().info(f"pwm-left: {self.left_pwm},right: {self.right_pwm}")

        # self.get_logger().info(f" prev_err_LEFT = {self.pid_left.prev_error}; integral_LEFT = {self.pid_left.integral}")
        # self.get_logger().info(f" prev_err = {self.pid_right.prev_error}; integral = {self.pid_right.integral}")

def main(args=None):
    rclpy.init(args=args)
    node = wheels_control()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
