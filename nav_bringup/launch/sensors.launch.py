import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav_bringup.global_config import FRAMES, GPS_ORIGIN_SIM, MAGNETIC_DECLINATION_RADIANS


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("nav_bringup")
    imu_params = PathJoinSubstitution([bringup_share, "config", "imu.yaml"])
    gps_params = PathJoinSubstitution([bringup_share, "config", "gps.yaml"])
    sensor_simulator_params = PathJoinSubstitution([bringup_share, "config", "sensor_simulator.yaml"])
    simulation = LaunchConfiguration("simulation")

    vectornav = Node(
        package="vectornav",
        executable="vectornav_node",
        name="vectornav",
        output="screen",
        condition=UnlessCondition(simulation),
        parameters=[
            imu_params,
            {"frame_id": FRAMES["imu_frame"]},
            {"map_frame_id": FRAMES["map_frame"]},
        ],
        remappings=[
            ("vectornav/data", "imu/raw"),
        ],
    )

    gps = Node(
        package="gps_publisher",
        executable="gps_publisher",
        name="gps_publisher",
        output="screen",
        condition=UnlessCondition(simulation),
        parameters=[
            gps_params,
            {"gps_frame_id": FRAMES["gps_frame"]},
        ],
        remappings=[
            ("gps", "gps/raw"),
        ],
    )

    sensor_simulator = Node(
        package="sensor_simulator",
        executable="sensor_simulator",
        name="sensor_simulator",
        output="screen",
        condition=IfCondition(simulation),
        parameters=[
            sensor_simulator_params,
            {
                "map_frame_id": FRAMES["map_frame"],
                "base_frame_id": FRAMES["base_frame"],
                "imu_frame_id": FRAMES["imu_frame"],
                "gps_frame_id": FRAMES["gps_frame"],
                "gps.origin_latitude_deg": GPS_ORIGIN_SIM["latitude"],
                "gps.origin_longitude_deg": GPS_ORIGIN_SIM["longitude"],
                "imu.magnetic_declination_rad": MAGNETIC_DECLINATION_RADIANS,
            },
        ],
        remappings=[
            ("gps", "gps/raw"),
            ("enc_vel", "enc_vel/raw"),
            ("imu", "imu/raw"),
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
            "--z", "0.8128",
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
            "--x", "0.00762",
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
            DeclareLaunchArgument(
                "simulation",
                default_value="false",
                description="Launch sensor simulator instead of real hardware drivers",
            ),
            vectornav,
            gps,
            sensor_simulator,
            tf_base_to_imu,
            tf_base_to_gps,
        ]
    )
