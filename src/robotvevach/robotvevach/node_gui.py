import tkinter as tk
from tkinter import ttk
import subprocess
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

# ------------------ ROS2 Node ------------------
class RobotInterfaceNode(Node):
    def __init__(self):
        super().__init__('robot_gui_node')
        self.status_sub = self.create_subscription(
            String, '/robot/status', self.status_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.device_status = "Unknown"

    def status_callback(self, msg):
        self.device_status = msg.data

    def set_param(self, node_name, param_name, value):
        client = self.create_client(SetParameters, f'{node_name}/set_parameters')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Waiting for {node_name}/set_parameters...')
        req = SetParameters.Request()
        param = Parameter()
        param.name = param_name
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = value
        req.parameters = [param]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

# ------------------ ROS Thread ------------------
def ros_thread():
    rclpy.init()
    node = RobotInterfaceNode()
    gui_app.set_ros_node(node)
    rclpy.spin(node)
    rclpy.shutdown()

# ------------------ GUI App ------------------
class RobotGUI:
    def __init__(self, root):
        self.root = root
        self.ros_node = None
        self.root.title("Futsal Line Robot GUI")
        self.tab_control = ttk.Notebook(root)

        self.home_tab = ttk.Frame(self.tab_control)
        self.control_tab = ttk.Frame(self.tab_control)
        self.config_tab = ttk.Frame(self.tab_control)

        self.tab_control.add(self.home_tab, text="Home")
        self.tab_control.add(self.control_tab, text="Robot Control")
        self.tab_control.add(self.config_tab, text="Configuration")
        self.tab_control.pack(expand=1, fill="both")

        self.build_home_tab()
        self.build_control_tab()
        self.build_config_tab()

        self.update_status()

    def set_ros_node(self, node):
        self.ros_node = node

    def build_home_tab(self):
        frame_map = ttk.LabelFrame(self.home_tab, text="Map Tools")
        frame_map.grid(row=0, column=0, padx=10, pady=10, sticky="ew")

        btn_rviz = ttk.Button(frame_map, text="Open RViz2", command=self.launch_rviz)
        btn_rviz.grid(row=0, column=0, padx=5, pady=5)

        btn_start_map = ttk.Button(frame_map, text="Start Mapping", command=self.launch_mapping)
        btn_start_map.grid(row=0, column=1, padx=5, pady=5)

        btn_load_map = ttk.Button(frame_map, text="Load Saved Map", command=self.launch_map_server)
        btn_load_map.grid(row=0, column=2, padx=5, pady=5)

        frame_teleop = ttk.LabelFrame(self.home_tab, text="Manual Control")
        frame_teleop.grid(row=1, column=0, padx=10, pady=10, sticky="ew")

        btn_teleop = ttk.Button(frame_teleop, text="Start Teleop", command=self.launch_teleop)
        btn_teleop.grid(row=0, column=0, padx=5, pady=5)

    def build_control_tab(self):
        frame_status = ttk.LabelFrame(self.control_tab, text="Peripheral Status")
        frame_status.grid(row=0, column=0, padx=10, pady=10, sticky="ew")

        self.status_label = ttk.Label(frame_status, text="Status: Unknown")
        self.status_label.grid(row=0, column=0, padx=5, pady=5)

        frame_cmd = ttk.LabelFrame(self.control_tab, text="Send Velocity")
        frame_cmd.grid(row=1, column=0, padx=10, pady=10)

        ttk.Label(frame_cmd, text="Linear X:").grid(row=0, column=0)
        ttk.Label(frame_cmd, text="Angular Z:").grid(row=1, column=0)
        self.lin_x = tk.DoubleVar()
        self.ang_z = tk.DoubleVar()
        tk.Entry(frame_cmd, textvariable=self.lin_x).grid(row=0, column=1)
        tk.Entry(frame_cmd, textvariable=self.ang_z).grid(row=1, column=1)
        ttk.Button(frame_cmd, text="Send", command=self.send_cmd_vel).grid(row=2, column=0, columnspan=2)

    def build_config_tab(self):
        frame_pid = ttk.LabelFrame(self.config_tab, text="Velocity PID Config")
        frame_pid.grid(row=0, column=0, padx=10, pady=10)
        self.kp_v = tk.DoubleVar()
        self.ki_v = tk.DoubleVar()
        self.kd_v = tk.DoubleVar()
        ttk.Label(frame_pid, text="Kp:").grid(row=0, column=0)
        ttk.Label(frame_pid, text="Ki:").grid(row=1, column=0)
        ttk.Label(frame_pid, text="Kd:").grid(row=2, column=0)
        tk.Entry(frame_pid, textvariable=self.kp_v).grid(row=0, column=1)
        tk.Entry(frame_pid, textvariable=self.ki_v).grid(row=1, column=1)
        tk.Entry(frame_pid, textvariable=self.kd_v).grid(row=2, column=1)
        ttk.Button(frame_pid, text="Apply", command=self.apply_pid_params).grid(row=3, column=0, columnspan=2)

        frame_pd = ttk.LabelFrame(self.config_tab, text="Pose PD Config")
        frame_pd.grid(row=1, column=0, padx=10, pady=10)
        self.kp_p = tk.DoubleVar()
        self.kd_p = tk.DoubleVar()
        ttk.Label(frame_pd, text="Kp:").grid(row=0, column=0)
        ttk.Label(frame_pd, text="Kd:").grid(row=1, column=0)
        tk.Entry(frame_pd, textvariable=self.kp_p).grid(row=0, column=1)
        tk.Entry(frame_pd, textvariable=self.kd_p).grid(row=1, column=1)
        ttk.Button(frame_pd, text="Apply", command=self.apply_pd_params).grid(row=2, column=0, columnspan=2)

    def launch_rviz(self):
        try:
            subprocess.Popen(["rviz2", "-d", "/path/to/your_config.rviz"])
        except Exception as e:
            print(f"Error launching RViz: {e}")

    def launch_mapping(self):
        subprocess.Popen(["ros2", "launch", "slam_toolbox", "online_async_launch.py"])

    def launch_map_server(self):
        subprocess.Popen(["ros2", "launch", "map_server", "map_server.launch.py"])

    def launch_teleop(self):
        subprocess.Popen(["ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard"])

    def send_cmd_vel(self):
        if self.ros_node:
            msg = Twist()
            msg.linear.x = self.lin_x.get()
            msg.angular.z = self.ang_z.get()
            self.ros_node.cmd_pub.publish(msg)

    def apply_pid_params(self):
        if self.ros_node:
            self.ros_node.set_param('/pid_controller', 'kp', self.kp_v.get())
            self.ros_node.set_param('/pid_controller', 'ki', self.ki_v.get())
            self.ros_node.set_param('/pid_controller', 'kd', self.kd_v.get())

    def apply_pd_params(self):
        if self.ros_node:
            self.ros_node.set_param('/pose_controller', 'kp', self.kp_p.get())
            self.ros_node.set_param('/pose_controller', 'kd', self.kd_p.get())

    def update_status(self):
        if self.ros_node:
            self.status_label.config(text=f"Status: {self.ros_node.device_status}")
        self.root.after(500, self.update_status)

# ------------------ Main ------------------
if __name__ == '__main__':
    root = tk.Tk()
    gui_app = RobotGUI(root)
    threading.Thread(target=ros_thread, daemon=True).start()
    root.mainloop()
