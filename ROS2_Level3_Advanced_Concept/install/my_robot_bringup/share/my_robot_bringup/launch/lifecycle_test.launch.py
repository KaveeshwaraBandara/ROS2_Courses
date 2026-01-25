from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    number_node_name = "my_number_publisher"

    number_node = LifecycleNode(
        package = 'lifecycle_py',
        executable = 'number_publisher',
        name = number_node_name,
        namespace=""
    )

    lifecycle_manager_node = Node(
        package = 'lifecycle_py',
        executable = 'lifecycle_node_manager',
        parameters=[{'manager_node_name': number_node_name}]
    )

    ld.add_action(number_node)
    ld.add_action(lifecycle_manager_node)
    
    return ld