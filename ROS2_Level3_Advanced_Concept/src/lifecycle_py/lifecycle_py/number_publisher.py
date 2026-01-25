#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode


from example_interfaces.msg import Int64
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

class NumberPublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__("number_publisher")
        self.get_logger().info("IN constructor")
        self.number_ = 1
        self.publish_frequency_ = 1.0
        self.number_publisher_ = None
        self.number_timer_ = None


       
        # self.get_logger().info("Number publisher has been started.")
    # create ROS2 communications, connect to HW
    def on_configure(self, state):
        self.get_logger().info("IN on_configure")
        self.number_publisher_ = self.create_lifecycle_publisher(Int64, "number", 10)
        self.number_timer_ = self.create_timer(
            1.0 / self.publish_frequency_, self.publish_number)
        self.number_timer_.cancel()
        return TransitionCallbackReturn.SUCCESS
    
    #activate/Enable HW
    def on_activate(self, state):
        self.get_logger().info("IN on_activate")
        self.number_timer_.reset()
        return super().on_activate(state)
    
    #deactivate/Disable HW
    def on_deactivate(self, state):
        self.get_logger().info("IN on_deactivate")
        self.number_timer_.cancel()
        return super().on_deactivate(state)

    #destroy ROS2 communications, disconnect from HW
    def on_cleanup(self, state):
        self.get_logger().info("IN on_cleanup")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return TransitionCallbackReturn.SUCCESS
    
    # cleanup everything
    def on_shutdown(self, state):
        self.get_logger().info("IN on_shutdown")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return TransitionCallbackReturn.SUCCESS
    
    # process errors, deactivate and cleanup
    def on_error(self, state):
        self.get_logger().info("IN on_error")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        # do some checks, if ok, then return SUCCESS, else FAILURE
        return TransitionCallbackReturn.SUCCESS
    
    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg)
        self.get_logger().info(f"published number: {msg.data}")
        self.number_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
