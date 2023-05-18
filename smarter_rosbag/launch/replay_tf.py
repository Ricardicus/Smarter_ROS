from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch rosbag play node
        Node(
            package='rosbag2',
            executable='play',
            arguments=['--ros-args', '-p', 'bag_path:=$(find smarter_rosbag)/tf.bag'],
            name='play'
        ),

        # Launch RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', '$(find smarter_rviz)/launch/smarter_tf.rviz'],
            name='rviz'
        ),
    ])
