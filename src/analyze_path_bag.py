#ros2 bag record /planned_path /odom_encoder -o robot_drawing_log

import rosbag2_py
import rclpy.serialization
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag_path = 'robot_drawing_log'

planned_path = []
actual_path = []

# Mở reader
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions('', '')
reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

type_map = {}

while reader.has_next():
    topic, data, t = reader.read_next()
    
    if topic not in type_map:
        type_map[topic] = get_message(topic)
    
    msg_type = get_message(topic)
    msg = deserialize_message(data, msg_type)

    if topic == '/planned_path':
        # chỉ lưu lần đầu (pose array không đổi)
        if not planned_path:
            for pose in msg.poses:
                planned_path.append((pose.position.x, pose.position.y))

    elif topic == '/odom_encoder':
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        actual_path.append((x, y))

# Chuyển sang numpy để plot
planned_path = np.array(planned_path)
actual_path = np.array(actual_path)

# Plot
plt.plot(planned_path[:, 0], planned_path[:, 1], label='Planned Path', color='green')
plt.plot(actual_path[:, 0], actual_path[:, 1], label='Actual Path', color='red')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.title('Quỹ đạo mong muốn vs thực tế')
plt.show()
