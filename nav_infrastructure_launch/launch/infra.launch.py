import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("nav_infrastructure_launch")
    waypoints_file = os.path.join(pkg_share, "config", "waypoints.json")

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
            ),
            Node(
                package="autonav_goal_selection",
                executable="autonav_goal_selection",
                name="autonav_goal_selection",
                parameters=[{"waypoints_file_path": waypoints_file}],
            ),
            Node(package="path_planning", executable="path_planning", name="path_planning"),
            Node(
                package="occupancy_grid_transform",
                executable="occupancy_grid_transform",
                name="occupancy_grid_transform",
                remappings=[
                    ("occupancy_grid", "occ_grid"),
                    ("occupancy_grid_transform", "inflated_occupancy_grid"),
                ],
            ),
            Node(package="path_tracking", executable="pure_pursuit", name="pure_pursuit"),
        ]
    )
