from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='turtlebot3_lab2',
            executable='turtle_control_node',
            name='turtle_control', 
            parameters=[
                {'distance_threshold': 0.5},
                {'clockwise_turn': True}
            ]
        )
    ])