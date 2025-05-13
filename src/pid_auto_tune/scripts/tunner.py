import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class PIDParamPublisher(Node):
    def __init__(self):
        super().__init__('pid_param_publisher')
        self.pub_linear = self.create_publisher(Float32MultiArray, '/pid_params_left', 10)
        self.pub_angular = self.create_publisher(Float32MultiArray, '/pid_params_right', 10)

        self.get_logger().info('Nhập hệ số PID. Nhấn Ctrl+C để thoát.\n')

        self.enter_and_publish()

    def enter_and_publish(self):
        while rclpy.ok():
            try:
                print("Nhập PID cho điều khiển *trai*:")
                kp_l = float(input("Kp: "))
                ki_l = float(input("Ki: "))
                kd_l = float(input("Kd: "))
                msg_l = Float32MultiArray()
                msg_l.data = [kp_l, ki_l, kd_l]
                self.pub_linear.publish(msg_l)

                print("Nhập PID cho điều khiển *phai*:")
                kp_a = float(input("Kp: "))
                ki_a = float(input("Ki: "))
                kd_a = float(input("Kd: "))
                msg_a = Float32MultiArray()
                msg_a.data = [kp_a, ki_a, kd_a]
                self.pub_angular.publish(msg_a)

                self.get_logger().info("Đã gửi PID mới!\n")

            except KeyboardInterrupt:
                print("Kết thúc.")
                break
            except Exception as e:
                self.get_logger().warn(f"Lỗi nhập liệu: {e}")

def main():
    rclpy.init()
    node = PIDParamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
