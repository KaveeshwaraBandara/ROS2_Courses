#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from my_robot_interfaces.action import CountUntil


class CountUntilClient(Node):
    def __init__(self):
        super().__init__("count_until_client")
        self.count_until_client_ = ActionClient(
            self,
            CountUntil,
            "count_until"
        )

    def send_goal(self, target_number, period):
        # wait for the action server to be available
        self.count_until_client_.wait_for_server()

        #create a goal message
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.period = period

        #send the goal to the action server
        self.get_logger().info(f"Sending goal: target_number={target_number}, period={period}")
        self.count_until_client_.send_goal_async(goal, feedback_callback=self.goal_feedback_callback).add_done_callback(self.goal_response_callback)

        # send a cancel request
        self.timer_ = self.create_timer(2.0, self.cancel_goal)

    def cancel_goal(self):
        self.get_logger().info("send a cancel request")
        self.goal_handle_.cancel_goal_async()
        self.timer_.cancel()

    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()

        if self.goal_handle_.accepted:
            self.goal_handle_.get_result_async().add_done_callback(self.get_result_callback)
            self.get_logger().info("Goal accepted by server, waiting for result...")
        else:
            self.get_logger().warn("Goal rejected by server")

    def get_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")

        self.get_logger().info(f"Result received: reached_number={result.reached_number}")
        
    def goal_feedback_callback(self, feedback_msg):
        number = feedback_msg.feedback.current_number
        self.get_logger().info(f"Got feedback: {number}")

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClient()
    node.send_goal(6,1.0)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()