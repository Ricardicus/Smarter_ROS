# Import necessary modules
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create a LaunchDescription object
    ld = LaunchDescription()

    # Add nodes to the LaunchDescription object
    ld.add_action(Node(package='rosbag', executable='play', name='play', arguments=['$(find smarter_rosbag)/tf.bag']))
    ld.add_action(Node(package='smarter_tf', executable='get_tfs', name='get_tfs'))
    ld.add_action(Node(package='rviz', executable='rviz', name='rviz', arguments=['-d', '$(find smarter_rviz)/launch/smarter_tf.rviz']))

    # Return the LaunchDescription object
    return ld
