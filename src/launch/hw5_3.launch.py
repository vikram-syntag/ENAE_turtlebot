from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='hw5',
            executable='hw5_3_control',
            name='turtle_control'
        )
    ])