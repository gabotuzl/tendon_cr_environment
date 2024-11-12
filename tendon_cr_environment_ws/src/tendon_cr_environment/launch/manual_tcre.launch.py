from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    simulator_node = Node(
        package="tendon_cr_environment",
        executable="simulator_node",
    )

    gui_manual_node = Node(
        package="tendon_cr_environment",
        executable="gui_manual_node",
    )

    visualizer_node = Node(
        package="tendon_cr_environment",
        executable="visualizer_node",
    )

    ld.add_action(simulator_node)
    ld.add_action(gui_manual_node)
    ld.add_action(visualizer_node)

    return ld