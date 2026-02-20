import math

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav_bringup.global_config import FRAMES


def generate_launch_description():
    bringup_share = FindPackageShare("nav_bringup")
    imu_params = PathJoinSubstitution([bringup_share, "config", "imu.yaml"])
    gps_params = PathJoinSubstitution([bringup_share, "config", "gps.yaml"])

    vectornav = Node(
        package="vectornav",
        executable="vectornav_node",
        name="vectornav",
        output="screen",
        parameters=[imu_params, {"frame_id": FRAMES["imu_frame"], "map_frame_id": FRAMES["map_frame"]}],
        remappings=[
            ("vectornav/data", "imu/raw"),
        ],
    )

    gps = Node(
        package="gps_publisher",
        executable="gps_publisher",
        name="gps_publisher",
        output="screen",
        parameters=[gps_params, {"gps_frame_id": FRAMES["gps_frame"]}],
        remappings=[
            ("gps", "gps/raw"),
        ],
    )

    # fmt: off
    tf_base_to_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_imu",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", str(-math.pi / 2),
            "--frame-id", FRAMES["base_frame"],
            "--child-frame-id", FRAMES["imu_frame"],
        ],
    )
    # fmt: on

    # fmt: off
    tf_base_to_gps = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_gps",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.8128",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", FRAMES["base_frame"],
            "--child-frame-id", FRAMES["gps_frame"],
        ],
    )
    # fmt: on

    return LaunchDescription(
        [
            vectornav,
            gps,
            tf_base_to_imu,
            tf_base_to_gps,
        ]
    )
