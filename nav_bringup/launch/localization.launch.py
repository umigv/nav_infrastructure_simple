from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav_bringup.global_config import FRAMES


def generate_launch_description():
    bringup_share = FindPackageShare("nav_bringup")
    localization_params = PathJoinSubstitution([bringup_share, "config", "localization.yaml"])

    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="screen",
        parameters=[
            localization_params,
            {
                "map_frame": FRAMES["map_frame"],
                "odom_frame": FRAMES["odom_frame"],
                "base_link_frame": FRAMES["base_frame"],
                "world_frame": FRAMES["odom_frame"],
            },
        ],
        remappings=[
            ("odometry/filtered", "odom/local"),
        ],
    )

    navsat = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[localization_params],
        remappings=[
            ("imu", "imu/raw"),
            ("gps/fix", "gps/raw"),
            ("odometry/filtered", "odom/local"),
            ("odometry/gps", "odom/gps"),
        ],
    )

    ekf_global = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global",
        output="screen",
        parameters=[
            localization_params,
            {
                "map_frame": FRAMES["map_frame"],
                "odom_frame": FRAMES["odom_frame"],
                "base_link_frame": FRAMES["base_frame"],
                "world_frame": FRAMES["map_frame"],
            },
        ],
        remappings=[
            ("odometry/filtered", "odometry/global"),
        ],
    )

    gps_origin_initializer = Node(
        package="localization",
        executable="gps_origin_initializer",
        name="gps_origin_initializer",
        output="screen",
        remappings=[("gps", "gps/raw"), ("set_datum", "datum")],
    )

    return LaunchDescription(
        [
            ekf_local,
            navsat,
            ekf_global,
            gps_origin_initializer,
        ]
    )
