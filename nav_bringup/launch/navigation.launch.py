from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav_bringup.global_config import FRAMES


def generate_launch_description() -> LaunchDescription:
    simulation_arg = DeclareLaunchArgument(
        "simulation", default_value="false", description="Whether to run in simulation mode"
    )

    return LaunchDescription(
        [
            simulation_arg,
            Node(
                package="point_simulator",
                executable="point_simulator",
                name="point_simulator",
                condition=IfCondition(LaunchConfiguration("simulation")),
                remappings=[
                    ("cmd_vel", "cmd_vel"),
                    ("odom", "odom/local"),
                    ("visualization_marker", "visualization_marker"),
                    ("gps_coords", "gps/raw"),
                ],
            ),
            Node(
                package="occupancy_grid_transform",
                executable="occupancy_grid_transform",
                name="occupancy_grid_transform",
                parameters=[
                    {"frame_id": FRAMES["odom_frame"]},
                ],
                remappings=[
                    ("occupancy_grid", "occ_grid"),
                    ("transformed_occupancy_grid", "inflated_occupancy_grid"),
                ],
            ),
            Node(
                package="goal_selection",
                executable="goal_selection",
                name="goal_selection",
                remappings=[
                    ("odom", "odom/local"),
                    ("gps_coords", "gps/raw"),
                    ("inflated_occupancy_grid", "inflated_occupancy_grid"),
                    ("path", "path"),
                ],
            ),
            Node(
                package="path_tracking",
                executable="path_tracking",
                name="path_tracking",
                parameters=[
                    {"base_frame_id": FRAMES["base_frame"]},
                    {"odom_frame_id": FRAMES["odom_frame"]},
                ],
                remappings=[
                    ("odom", "odom/local"),
                    ("path", "path"),
                    ("nav_cmd_vel", "nav_cmd_vel"),
                    ("smoothed_path", "smoothed_path"),
                ],
            ),
        ]
    )
