#!usr/bin/env python3
import rclpy
import time 
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

class LifecycleNodeManager(Node):
    def __init__(self):
        super().__init__("lifecycle_manager")
        self.declare_parameter("manager_node_name", rclpy.Parameter.Type.STRING)
        node_name = self.get_parameter("manager_node_name").value
        service_change_state_name = "/" + node_name + "/change_state"
        self.client = self.create_client(ChangeState, service_change_state_name)

    def change_state(self, transition: Transition):
        self.client.wait_for_service()
        request = ChangeState.Request()
        request.transition = transition
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def initialization_sequence(self):
        self.get_logger().info("Starting initialization sequence...")
        transition = Transition()
        transition.id = Transition.TRANSITION_CONFIGURE
        transition.label = "configure"
        self.change_state(transition)
        self.get_logger().info("Node configured.")
        time.sleep(3)

        # Activate
        self.get_logger().info("Activating node...")
        transition = Transition()
        transition.id = Transition.TRANSITION_ACTIVATE
        transition.label = "activate"
        self.change_state(transition)
        self.get_logger().info("Node activated.")

def main(args=None):
    rclpy.init(args=args)
    lifecycle_node_manager = LifecycleNodeManager()
    lifecycle_node_manager.initialization_sequence()
    rclpy.shutdown()
