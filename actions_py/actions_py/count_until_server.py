#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import CountUntil

class CountUntilServerNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("count_until_server") # MODIFY NAME
        self.count_until_server_ = ActionServer(self,
            CountUntil,
            "count_until",
            goal_callback= self.goal_callback,
            execute_callback=self.execute_callback)
        self.get_logger().info("Action Server has started......")
        
    def goal_callback(self, goal_request: CountUntil.Goal) : 
        self.get_logger().info("Goal Received")
        if goal_request.target_number < 0:
            self.get_logger().warning("GOAL REJECTED!!!")
            return GoalResponse.REJECT
        self.get_logger().info("GOAL ACCEPTED!!!")
        return GoalResponse.ACCEPT
    def execute_callback(self, goal_handle: ServerGoalHandle):
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        self.get_logger().info("Executing the Goal!!!!!!!!")
        counter = 0

        for i in range(target_number):
            counter +=1
            self.get_logger().info("Counter is : "+ str(counter))
            time.sleep(period)
        
        goal_handle.succeed()

        result = CountUntil.Result()
        result.reached_number = counter
        return result



def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()