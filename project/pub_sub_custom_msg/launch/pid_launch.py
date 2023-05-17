# pub_sub_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
        package='pub_sub_custom_msg', # Package Name
        executable='pid_controller', # Executable file
        output='screen',
        emulate_tty=True),
    ])
