#colcon build --packages-select robot_actions
#source install/setup.bash
# ros2 run robot_actions robot_action_server chay action
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot_actions.action import RobotActions
import math

class RobotActionServer(Node):
    def __init__(self):
        super().__init__('robot_action_server')
        
        # Khởi tạo action servers
        self._move_straight_server = ActionServer(
            self,
            RobotActions.MoveStraight,
            'move_straight',
            self.move_straight_callback)
            
        self._move_circle_server = ActionServer(
            self,
            RobotActions.MoveCircle,
            'move_circle',
            self.move_circle_callback)
            
        self._pump_control_server = ActionServer(
            self,
            RobotActions.PumpControl,
            'pump_control',
            self.pump_control_callback)
            
        # Publisher để điều khiển robot
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber để nhận odometry
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
            
        # Biến lưu trữ trạng thái hiện tại
        self.current_position = None
        self.current_orientation = None
        self.pump_status = False
        
    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation
        
    async def move_straight_callback(self, goal_handle):
        self.get_logger().info('Executing move straight...')
        feedback_msg = RobotActions.MoveStraight.Feedback()
        result = RobotActions.MoveStraight.Result()
        
        start_position = self.current_position
        distance_to_go = goal_handle.request.distance
        distance_traveled = 0.0
        
        # Tạo velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.4  # Tốc độ 0.4 m/s
        
        while distance_traveled < distance_to_go:
            # Kiểm tra có bị cancel không
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                self.get_logger().info('Move straight canceled')
                cmd_vel.linear.x = 0.0
                self.cmd_vel_publisher.publish(cmd_vel)
                return result
                
            # Tính toán quãng đường đã đi
            distance_traveled = math.sqrt(
                (self.current_position.x - start_position.x)**2 +
                (self.current_position.y - start_position.y)**2)
                
            # Gửi feedback
            feedback_msg.remaining_distance = distance_to_go - distance_traveled
            goal_handle.publish_feedback(feedback_msg)
            
            # Gửi lệnh di chuyển
            self.cmd_vel_publisher.publish(cmd_vel)
            
            # Chờ một chút
            await asyncio.sleep(0.1)
            
        # Dừng robot
        cmd_vel.linear.x = 0.0
        self.cmd_vel_publisher.publish(cmd_vel)
        
        result.success = True
        goal_handle.succeed()
        return result
        
    async def move_circle_callback(self, goal_handle):
        self.get_logger().info('Executing move circle...')
        feedback_msg = RobotActions.MoveCircle.Feedback()
        result = RobotActions.MoveCircle.Result()
        
        radius = goal_handle.request.radius
        target_angle = goal_handle.request.angle
        angle_traveled = 0.0
        
        # Tính toán vận tốc góc và tuyến tính
        linear_speed = 0.2  # m/s
        angular_speed = linear_speed / radius  # rad/s
        
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_speed
        cmd_vel.angular.z = angular_speed
        
        start_orientation = self.current_orientation
        # Chuyển đổi orientation sang góc yaw (đơn giản hóa)
        start_yaw = self.quaternion_to_yaw(start_orientation)
        
        while angle_traveled < target_angle:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                self.get_logger().info('Move circle canceled')
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.cmd_vel_publisher.publish(cmd_vel)
                return result
                
            current_yaw = self.quaternion_to_yaw(self.current_orientation)
            angle_traveled = abs(current_yaw - start_yaw)
            
            feedback_msg.remaining_angle = target_angle - angle_traveled
            goal_handle.publish_feedback(feedback_msg)
            
            self.cmd_vel_publisher.publish(cmd_vel)
            await asyncio.sleep(0.1)
            
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_vel)
        
        result.success = True
        goal_handle.succeed()
        return result
        
    async def pump_control_callback(self, goal_handle):
        self.get_logger().info('Executing pump control...')
        feedback_msg = RobotActions.PumpControl.Feedback()
        result = RobotActions.PumpControl.Result()
        
        turn_on = goal_handle.request.turn_on
        
        
        # pub 
        msg = Int8()
        msg.data = 1 if turn_on else 0
        self.son_publisher.publish(msg)
        self.get_logger().info(f'Published {msg.data} to /son topic')
        self.pump_status = turn_on
        
        feedback_msg.is_running = self.pump_status
        goal_handle.publish_feedback(feedback_msg)
        
        result.success = True
        goal_handle.succeed()
        return result
        
    def quaternion_to_yaw(self, q):
        # Chuyển đổi quaternion sang góc yaw (đơn giản hóa)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    robot_action_server = RobotActionServer()
    rclpy.spin(robot_action_server)
    robot_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
