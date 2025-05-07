import sys
from PyQt5.QtWidgets import QApplication, QWidget, QSlider, QLabel, QVBoxLayout
from PyQt5.QtCore import Qt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class KSliderNode(Node):
    def __init__(self):
        super().__init__('k_tuner_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/k_values', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.kx = 1.0
        self.ktheta = 3.0

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = [self.kx, self.ktheta]
        self.publisher_.publish(msg)

class KSliderGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle('🎛️ Kx & Kθ Tuner')
        self.setGeometry(100, 100, 300, 150)

        layout = QVBoxLayout()

        # Kx Slider
        self.kx_label = QLabel(f'Kx: {self.ros_node.kx:.2f}', self)
        self.kx_slider = QSlider(Qt.Horizontal)
        self.kx_slider.setMinimum(0)
        self.kx_slider.setMaximum(100)
        self.kx_slider.setValue(int(self.ros_node.kx * 10))
        self.kx_slider.valueChanged.connect(self.update_kx)

        # Ktheta Slider
        self.ktheta_label = QLabel(f'Kθ: {self.ros_node.ktheta:.2f}', self)
        self.ktheta_slider = QSlider(Qt.Horizontal)
        self.ktheta_slider.setMinimum(0)
        self.ktheta_slider.setMaximum(100)
        self.ktheta_slider.setValue(int(self.ros_node.ktheta * 10))
        self.ktheta_slider.valueChanged.connect(self.update_ktheta)

        layout.addWidget(self.kx_label)
        layout.addWidget(self.kx_slider)
        layout.addWidget(self.ktheta_label)
        layout.addWidget(self.ktheta_slider)
        self.setLayout(layout)

    def update_kx(self, value):
        self.ros_node.kx = value / 10.0
        self.kx_label.setText(f'Kx: {self.ros_node.kx:.2f}')

    def update_ktheta(self, value):
        self.ros_node.ktheta = value / 10.0
        self.ktheta_label.setText(f'Kθ: {self.ros_node.ktheta:.2f}')

def main():
    rclpy.init()
    ros_node = KSliderNode()

    app = QApplication(sys.argv)
    gui = KSliderGUI(ros_node)
    gui.show()

    # Spin ROS2 in background
    from threading import Thread
    ros_thread = Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    sys.exit(app.exec_())

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
