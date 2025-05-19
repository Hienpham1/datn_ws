import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_actions.action import RobotActions
import asyncio

class SequentialActions(Node):
    def __init__(self):
        super().__init__('sequential_actions')
        
        # Tạo clients cho các action
        self.move_straight_client = ActionClient(
            self, RobotActions.MoveStraight, 'move_straight')
        self.move_circle_client = ActionClient(
            self, RobotActions.MoveCircle, 'move_circle')
        self.pump_control_client = ActionClient(
            self, RobotActions.PumpControl, 'pump_control')
            
    async def run_sequence(self):
        # 1. Bật bơm
        await self.execute_pump_action(True)
        
        # 2. Di chuyển thẳng 1m
        await self.execute_move_action(distance=1.0)
        
        # 3. Tắt bơm
        await self.execute_pump_action(False)
        
        # Hoặc chuỗi khác: bật bơm -> di chuyển 90 độ -> tắt bơm
        await self.execute_pump_action(True)
        await self.execute_circle_action(radius=2.0, angle=1.57)  # 90 độ = 1.57 rad
        await self.execute_pump_action(False)
        
    async def execute_pump_action(self, turn_on):
        goal = RobotActions.PumpControl.Goal()
        goal.turn_on = turn_on
        
        self.pump_control_client.wait_for_server()
        goal_handle = await self.pump_control_client.send_goal_async(goal)
        result = await goal_handle.get_result_async()
        return result.success
        
    async def execute_move_action(self, distance):
        goal = RobotActions.MoveStraight.Goal()
        goal.distance = distance
        
        self.move_straight_client.wait_for_server()
        goal_handle = await self.move_straight_client.send_goal_async(goal)
        result = await goal_handle.get_result_async()
        return result.success
        
    async def execute_circle_action(self, radius, angle):
        goal = RobotActions.MoveCircle.Goal()
        goal.radius = radius
        goal.angle = angle
        
        self.move_circle_client.wait_for_server()
        goal_handle = await self.move_circle_client.send_goal_async(goal)
        result = await goal_handle.get_result_async()
        return result.success

async def main(args=None):
    rclpy.init(args=args)
    sequential_actions = SequentialActions()
    
    # Chạy chuỗi hành động
    await sequential_actions.run_sequence()
    
    sequential_actions.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())
