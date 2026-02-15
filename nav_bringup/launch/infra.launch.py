from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    simulation_arg = DeclareLaunchArgument(
        "simulation", default_value="false", description="Whether to run in simulation mode"
    )
    
    mode_arg = DeclareLaunchArgument(
        "mode", default_value="auto_nav", description="Navigation mode: 'auto_nav' (GPS-based waypoints) or 'self_drive' (Point-based goals)"
    )

    return LaunchDescription(
        [
            simulation_arg,
            mode_arg,
            Node(
                package="point_simulator",
                executable="point_simulator",
                name="point_simulator",
                condition=IfCondition(LaunchConfiguration("simulation")),
            ),
            # auto_nav mode: GPS-based goal selection node
            Node(
                package="goal_selection",
                executable="auto_nav",
                name="goal_selection",
                condition=LaunchConfigurationEquals("mode", "auto_nav"),
            ),
            # Self-drive mode: Point-based goal selection node
            Node(
                package="goal_selection",
                executable="self_drive",
                name="goal_selection",
                condition=LaunchConfigurationEquals("mode", "self_drive"),
            ),
            Node(
                package="occupancy_grid_transform",
                executable="occupancy_grid_transform",
                name="occupancy_grid_transform",
                remappings=[
                    ("occ_grid/self_drive", "occ_grid"),
                    ("transformed_occupancy_grid", "inflated_occupancy_grid"),
                ],
            ),
            Node(package="path_tracking", executable="pure_pursuit", name="pure_pursuit"),
        ]
    )