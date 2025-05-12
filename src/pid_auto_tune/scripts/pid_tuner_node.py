import rclpy
from rclpy.node import Node
import numpy as np
import math
import os
import yaml

def fix_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def heading_error(pose, target):
    dx = target[0] - pose[0]
    dy = target[1] - pose[1]
    desired_heading = math.atan2(dy, dx)
    current_heading = pose[2]
    return fix_angle(desired_heading - current_heading)

def simulate_pid(path, odom_data, Kp, Ki, Kd, dt=0.05):
    E = 0.0
    old_e = 0.0
    mse = 0.0
    count = 0

    for i in range(min(len(path), len(odom_data))):
        x, y, theta = odom_data[i]
        gx, gy = path[i]

        e_P = heading_error((x, y, theta), (gx, gy))
        E += e_P * dt
        e_D = (e_P - old_e) / dt
        old_e = e_P

        w = Kp * e_P + Ki * E + Kd * e_D
        mse += e_P**2
        count += 1

    return mse / count if count > 0 else float('inf')

def estimate_heading(odom_raw):
    headings = []
    for i in range(len(odom_raw)):
        if i < 2:
            headings.append(0.0)
        else:
            dx = odom_raw[i][1] - odom_raw[i - 1][1]
            dy = odom_raw[i][2] - odom_raw[i - 1][2]
            headings.append(math.atan2(dy, dx))
    return np.hstack((odom_raw[:, 1:], np.array(headings).reshape(-1, 1)))

class PIDTunerNode(Node):
    def __init__(self):
        super().__init__('pid_tuner_node')
        self.declare_parameter("data_file", "data/data_log.npz")
        self.declare_parameter("output_file", "config/tuned_pid.yaml")

        data_file = self.get_parameter("data_file").get_parameter_value().string_value
        output_file = self.get_parameter("output_file").get_parameter_value().string_value

        npz_path = os.path.join(os.path.dirname(__file__), '..', data_file)
        yaml_path = os.path.join(os.path.dirname(__file__), '..', output_file)

        if not os.path.exists(npz_path):
            self.get_logger().error(f"Không tìm thấy file dữ liệu: {npz_path}")
            return

        data = np.load(npz_path)
        path = data['path']
        odom_raw = data['odom']
        odom_data = estimate_heading(odom_raw)

        best_pid, best_mse = self.grid_search_pid(path, odom_data)
        self.save_to_yaml(best_pid, yaml_path)

        self.get_logger().info(f"PID tốt nhất: {best_pid}, MSE={best_mse:.6f}")

    def grid_search_pid(self, path, odom_data):
        best_mse = float('inf')
        best_params = {'Kp': 0.0, 'Ki': 0.0, 'Kd': 0.0}

        Kp_range = np.arange(0.5, 5.0, 0.5)
        Ki_range = np.arange(0.0, 0.1, 0.01)
        Kd_range = np.arange(0.0, 1.0, 0.1)

        for Kp in Kp_range:
            for Ki in Ki_range:
                for Kd in Kd_range:
                    mse = simulate_pid(path, odom_data, Kp, Ki, Kd)
                    if mse < best_mse:
                        best_mse = mse
                        best_params = {'Kp': float(Kp), 'Ki': float(Ki), 'Kd': float(Kd)}
        return best_params, best_mse

    def save_to_yaml(self, params, file_path):
        with open(file_path, 'w') as f:
            yaml.dump(params, f)
        self.get_logger().info(f"Đã lưu PID tốt nhất vào {file_path}")


def main(args=None):
    rclpy.init(args=args)
    node = PIDTunerNode()
    node.destroy_node()
    rclpy.shutdown()
