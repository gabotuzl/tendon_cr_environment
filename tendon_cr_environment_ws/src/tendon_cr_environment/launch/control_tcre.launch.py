from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    simulator_node = Node(
        package="tendon_cr_environment",
        executable="simulator_node",
    )

    gui_control_node = Node(
        package="tendon_cr_environment",
        executable="gui_control_node",
    )

    controller_node = Node(
        package="tendon_cr_environment",
        executable="controller_node",
    )

    gain_searcher_node = Node(
        package="tendon_cr_environment",
        executable="gain_searcher_node",
    )

    visualizer_node = Node(
        package="tendon_cr_environment",
        executable="visualizer_node",
    )

    ld.add_action(simulator_node)
    ld.add_action(gui_control_node)
    ld.add_action(controller_node)
    ld.add_action(gain_searcher_node)
    ld.add_action(visualizer_node)

    return ld