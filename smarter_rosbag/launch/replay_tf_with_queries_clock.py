# Import necessary modules
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create a LaunchDescription object
    ld = LaunchDescription()

    # Add nodes to the LaunchDescription object
    ld.add_action(Node(package='rosbag', executable='play', arguments=['--clock', 'tf.bag'], parameters=[{'use_sim_time': True}]))
    ld.add_action(Node(package='smarter_tf', executable='get_tfs'))
    ld.add_action(Node(package='rviz', executable='rviz', arguments=['-d', 'launch/smarter_tf.rviz']))

    # Return the LaunchDescription object
    return ld
