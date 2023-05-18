# Import necessary modules
from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions
import launch.substitutions

def generate_launch_description():
    # Create a LaunchDescription object
    ld = LaunchDescription()

    # Add send_tfs node
    send_tfs_node = Node(
        package="smarter_tf",
        executable="send_tfs",
        name="send_tfs"
    )
    ld.add_action(send_tfs_node)

    # Add record node
    record_node = Node(
        package="rosbag2",
        executable="record",
        name="record",
        arguments=["-o", launch.substitutions.PathJoinSubstitution(["$(find smarter_rosbag)", "tf.bag"]), "/tf"]
    )
    ld.add_action(record_node)

    # Add rviz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", launch.substitutions.PathJoinSubstitution(["$(find smarter_rviz)", "launch", "smarter_tf.rviz"])]
    )
    ld.add_action(rviz_node)

    return ld
