#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import CountUntil
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class CountUntilServer(Node):
    def __init__(self):
        super().__init__("count_until_server")
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.goal_queue_ = []
        self.count_until_server_ = ActionServer(
            self,
            CountUntil, 
            "count_until",
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info("Action server has been started")
        
    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info(f"Received goal request: target_number={goal_request.target_number}, period={goal_request.period}")

        # Policy: refuse new goal if current goal still active
        # with self.goal_lock_:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().info("Rejecting the goal")
        #         return GoalResponse.REJECT



        # validate goal request
        if goal_request.target_number <= 0 or goal_request.period <= 0.0:
            self.get_logger().info("Rejected goal request")
            self.get_logger().info("target_number and period must be positive values")
            return GoalResponse.REJECT
        
        # Policy: preempt existing goal when receiving new goal
        # with self.goal_lock_:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.goal_handle_.abort()
        #         self.get_logger().info("Abort current goal and acept new goal")

        
        self.get_logger().info("Accepting the Goal")
        return GoalResponse.ACCEPT
    
    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            if self.goal_handle_ is not None:
                self.goal_queue_.append(goal_handle)
            else:
                goal_handle.execute()
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_ = goal_handle
        # Get request from goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period
        self.get_logger().info(f"Received goal request: target_number={target_number}, period={period}")

        # Execute the action
        self.get_logger().info("Executing goal...")
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()
        counter = 0
        for i in range(target_number):
            if not goal_handle.is_active:
                result.reached_number = counter
                self.process_next_goal()
                return result
            
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancelling the goal")
                goal_handle.canceled()
                result.reached_number = counter
                return result
            counter+=1
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"Counting: {counter}")
             # Check if there is a cancel request
            time.sleep(period)

        # once done, set goal final state
        goal_handle.succeed()

        # and send the result
        
        result.reached_number = counter
        self.process_next_goal()
        return result

    def process_next_goal(self):
        with self.goal_lock_:
            if len(self.goal_queue_) > 0:
                self.goal_queue_.pop(0).execute()
            else:
                self.goal_handle_ = None


def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServer()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == "__main__":
    main()