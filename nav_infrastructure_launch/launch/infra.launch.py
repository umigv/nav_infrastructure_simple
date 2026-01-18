from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulate_gps_arg = DeclareLaunchArgument(
        'simulate_gps',
        default_value='false',
        description='Whether to simulate GPS data from odometry'
    )

    return LaunchDescription([
        simulate_gps_arg,
        Node(
            package='goal_selection',
            executable='goal_selection',
            name='goal_selection',
            parameters=[{
                'simulate_gps': LaunchConfiguration('simulate_gps')
            }]
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