from launch import LaunchDescription
from launch_ros.actions import Node
from nav_bringup.global_config import FRAMES


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
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
