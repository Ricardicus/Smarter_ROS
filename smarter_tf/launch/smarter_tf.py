from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch send_tfs node from smarter_tf package
        Node(
            package='smarter_tf',
            executable='send_tfs',
            name='send_tfs'
        ),
        # Launch get_tfs node from smarter_tf package
        Node(
            package='smarter_tf',
            executable='get_tfs',
            name='get_tfs'
        ),
        # Launch rviz with the specified configuration file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', 'install/smarter_rviz/share/smarter_rviz/launch/smarter_tf.rviz']
        ),
    ])
