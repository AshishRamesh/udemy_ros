#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import CountUntil
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class CountUntilServerNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("count_until_server") # MODIFY NAME
        self.count_until_server_ = ActionServer(self,
            CountUntil,
            "count_until",
            goal_callback= self.goal_callback,
            cancel_callback= self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info("Action Server has started......")
        
    def goal_callback(self, goal_request: CountUntil.Goal) : 
        self.get_logger().info("Goal Received")
        if goal_request.target_number < 0:
            self.get_logger().warning("GOAL REJECTED!!!")
            return GoalResponse.REJECT
        self.get_logger().info("GOAL ACCEPTED!!!")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a cancel Request")
        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        self.get_logger().info("Executing the Goal!!!!!!!!")
        counter = 0
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()

        for i in range(target_number):
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancelling Goal....")
                goal_handle.canceled()
                result.reached_number = counter 
                return result
            counter +=1
            self.get_logger().info("Counter is : "+ str(counter))
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            time.sleep(period)
        
        goal_handle.succeed()

        
        result.reached_number = counter
        return result



def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode() # MODIFY NAME
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()