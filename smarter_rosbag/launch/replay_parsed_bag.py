import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get paths to packages
    smarter_rosbag_dir = get_package_share_directory('smarter_rosbag')
    smarter_rviz_dir = get_package_share_directory('smarter_rviz')

    # Create launch description
    ld = LaunchDescription([
        # Set use_sim_time parameter
        Node(
            package='ros2param',
            executable='set',
            arguments=['/use_sim_time', 'true'],
            output='screen'
        ),

        # Play rosbag
        Node(
            package='rosbag2',
            executable='play',
            arguments=['--clock', '-r', '4', os.path.join(smarter_rosbag_dir, 'data', 'data.log.bag')],
            output='screen'
        ),

        # Start RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(smarter_rviz_dir, 'launch', 'replay_parsed_bag.rviz')],
            output='screen'
        ),

        # Create static transform between laser scanner and robot
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['1', '0', '0', '3.14159', '0', '0', 'state_base_link', 'laserscan0'],
            output='screen'
        ),
    ])

    return ld
