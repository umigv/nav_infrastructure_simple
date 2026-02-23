from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav_bringup.global_config import FRAMES


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("nav_bringup")
    waypoints_file = PathJoinSubstitution([bringup_share, "config", "waypoints.json"])

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
                    ("transformed_occupancy_grid", "occupancy_grid"),
                ],
            ),
            Node(
                package="autonav_goal_selection",
                executable="autonav_goal_selection",
                name="autonav_goal_selection",
                parameters=[
                    {"waypoints_file_path": waypoints_file},
                    {"map_frame_id": FRAMES["map_frame"]},
                    {"world_frame_id": FRAMES["odom_frame"]},
                ],
                remappings=[
                    ("occupancy_grid", "occupancy_grid"),
                    ("odom", "odom/local"),
                    ("localization_initialized", "localization_initialized"),
                    ("fromLL", "fromLL"),
                    ("goal", "goal"),
                    ("gps_waypoint", "gps_waypoint"),
                ],
            ),
            Node(
                package="path_planning",
                executable="path_planning",
                name="path_planning",
                parameters=[
                    {"frame_id": FRAMES["odom_frame"]},
                ],
                remappings=[
                    ("occupancy_grid", "occupancy_grid"),
                    ("odom", "odom/local"),
                    ("goal", "goal"),
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
                ],
            ),
        ]
    )
