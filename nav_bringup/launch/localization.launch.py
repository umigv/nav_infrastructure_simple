from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav_bringup.global_config import FRAMES


def generate_launch_description():
    sim_arg = DeclareLaunchArgument("sim", default_value="false", description="Run in simulation mode")
    sim = LaunchConfiguration("sim")

    bringup_share = FindPackageShare("nav_bringup")
    localization_params = PathJoinSubstitution([bringup_share, "config", "localization.yaml"])

    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="screen",
        condition=UnlessCondition(sim),
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
        condition=UnlessCondition(sim),
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
        condition=UnlessCondition(sim),
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
            ("odometry/filtered", "odom/global"),
        ],
    )

    gps_origin_initializer = Node(
        package="localization",
        executable="gps_origin_initializer",
        name="gps_origin_initializer",
        output="screen",
        condition=UnlessCondition(sim),
        remappings=[("gps", "gps/raw"), ("set_datum", "datum")],
    )

    localization_simulator = Node(
        package="localization_simulator",
        executable="localization_simulator",
        name="localization_simulator",
        condition=IfCondition(sim),
        parameters=[
            {
                "map_frame_id": FRAMES["map_frame"],
                "odom_frame_id": FRAMES["odom_frame"],
                "base_frame_id": FRAMES["base_frame"],
            },
        ],
    )

    return LaunchDescription(
        [
            sim_arg,
            ekf_local,
            navsat,
            ekf_global,
            gps_origin_initializer,
            localization_simulator,
        ]
    )
