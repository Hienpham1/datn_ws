import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class PIDParamPublisher(Node):
    def __init__(self):
        super().__init__('pid_param_publisher')
        self.pub_left = self.create_publisher(Float32MultiArray, '/pid_params_left', 10)
        self.pub_right = self.create_publisher(Float32MultiArray, '/pid_params_right', 10)
        self.pub_theta = self.create_publisher(Float32MultiArray, '/pid_params_theta', 10)

        self.get_logger().info('Nhập hệ số PID. Nhấn Ctrl+C để thoát.\n')

        self.enter_and_publish()

    def enter_and_publish(self):
        while rclpy.ok():
            try:
                print("Nhap PID cho banh trai:")
                kp_l = float(input("Kp: "))
                ki_l = float(input("Ki: "))
                kd_l = float(input("Kd: "))
                msg_l = Float32MultiArray()
                msg_l.data = [kp_l, ki_l, kd_l]
                self.pub_left.publish(msg_l)

                # print("Nhap PID cho banh phai:")
                # kp_a = float(input("Kp: "))
                # ki_a = float(input("Ki: "))
                # kd_a = float(input("Kd: "))
                # msg_a = Float32MultiArray()
                # msg_a.data = [kp_a, ki_a, kd_a]
                # self.pub_right.publish(msg_a)

                # print("Nhap PID cho banh goc:")
                # kp_theta = float(input("Kp: "))
                # ki_theta = float(input("Ki: "))
                # kd_theta = float(input("Kd: "))
                # msg_theta = Float32MultiArray()
                # msg_theta.data = [kp_theta, ki_theta, kd_theta]
                # self.pub_theta.publish(msg_theta)

                self.get_logger().info("Da gui PID qua bo dieu khien!\n")

            except KeyboardInterrupt:
                print("Ket thuc")
                break
            except Exception as e:
                self.get_logger().warn(f"loi: {e}")

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
