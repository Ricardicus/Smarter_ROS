# sub_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
        package='pub_sub_custom_msg', # package name
        executable='subscriber', # executable file
        output='screen',
        emulate_tty=True),
    ])
