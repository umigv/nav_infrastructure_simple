from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
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
