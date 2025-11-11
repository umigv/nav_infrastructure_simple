from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='goal_selection',
            executable='goal_selection',
            name='goal_selection'
        ),
        Node(
            package='occupancy_grid_inflation',
            executable='inflation_node',
            name='inflation_node'
        ),
        Node(
            package='path_tracking',
            executable='pure_pursuit',
            name='pure_pursuit'
        )
    ])