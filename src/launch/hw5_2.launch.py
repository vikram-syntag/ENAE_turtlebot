from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='hw5',
            executable='hw5_2_control',
            name='turtle_control', 
            parameters=[
                {'distance_threshold': 0.5},
                {'clockwise_turn': False}
            ]
        )
    ])