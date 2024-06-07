#!/usr/bin/env python3
import rclpy
import time , threading
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import CountUntil
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class CountUntilServerNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("count_until_server") # MODIFY NAME
        self.goal_handle_ = ServerGoalHandle = None 
        self.goal_lock_ = threading.Lock()
        self.goal_queue_ = []
        self.count_until_server_ = ActionServer(self,
            CountUntil,
            "count_until",
            goal_callback= self.goal_callback,
            handle_accepted_callback= self.handle_accepted_callback,
            cancel_callback= self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info("Action Server has started......")
        
    def goal_callback(self, goal_request: CountUntil.Goal) : 
        self.get_logger().info("Goal Received")

        # with self.goal_lock_:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().info("A goal is already active !!!")
        #         return GoalResponse.REJECT
        if goal_request.target_number < 0:
            self.get_logger().warning("GOAL REJECTED!!!")
            return GoalResponse.REJECT

        # with self.goal_lock_:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().info("Aborting current goal and accepting new goal ")
        #         self.goal_handle_.abort()
        #         

        self.get_logger().info("GOAL ACCEPTED!!!")
        return GoalResponse.ACCEPT
    
    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            if self.goal_handle_ is not None:
                self.goal_queue_.append(goal_handle)
            else:
                goal_handle.execute()

    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a cancel Request")
        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        self.get_logger().info("Executing the Goal!!!!!!!!")
        counter = 0
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()

        for i in range(target_number):
            if not goal_handle.is_active:
                result.reached_number = counter 
                self.process_next_goal()
                return result
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancelling Goal....")
                goal_handle.canceled()
                result.reached_number = counter 
                self.process_next_goal()
                return result
            counter +=1
            self.get_logger().info("Counter is : "+ str(counter))
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            time.sleep(period)
        
        goal_handle.succeed()

        
        result.reached_number = counter
        self.process_next_goal()
        return result
    
    def process_next_goal(self):
        with self.goal_lock_:
            if len(self.goal_queue_)>0:
                self.goal_queue_.pop(0).execute()
            else:
                self.goal_handle_ = None


def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode() # MODIFY NAME
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()