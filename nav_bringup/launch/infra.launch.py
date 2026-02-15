from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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
            Node(package="goal_selection", executable="goal_selection", name="goal_selection"),
            Node(
                package="occupancy_grid_transform",
                executable="occupancy_grid_transform",
                name="occupancy_grid_transform",
                remappings=[
                    ("occupancy_grid", "occ_grid"),
                    ("transformed_occupancy_grid", "inflated_occupancy_grid"),
                ],
            ),
            Node(package="path_tracking", executable="pure_pursuit", name="pure_pursuit"),
        ]
    )
