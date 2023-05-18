# Import necessary modules
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create a LaunchDescription object
    ld = LaunchDescription()

    # Add send_rviz_markers node to the launch description
    send_rviz_markers_node = Node(
        package="smarter_rviz",
        executable="send_rviz_markers",
        name="send_rviz_markers"
    )
    ld.add_action(send_rviz_markers_node)

    # Add rviz node to the launch description
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", "$(find smarter_rviz)/launch/smarter_rviz.rviz"]
    )
    ld.add_action(rviz_node)

    return ld
