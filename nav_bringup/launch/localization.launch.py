from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav_bringup.global_config import FRAMES, GPS_ORIGIN_SIM


def generate_launch_description() -> LaunchDescription:
    simulation_arg = DeclareLaunchArgument("simulation", default_value="false", description="Run in simulation mode")
    simulation = LaunchConfiguration("simulation")

    localization_simulator = Node(
        package="localization_simulator",
        executable="localization_simulator",
        name="localization_simulator",
        condition=IfCondition(simulation),
        parameters=[
            {
                "map_frame_id": FRAMES["map_frame"],
                "odom_frame_id": FRAMES["odom_frame"],
                "base_frame_id": FRAMES["base_frame"],
                "gps_origin_latitude": GPS_ORIGIN_SIM["latitude"],
                "gps_origin_longitude": GPS_ORIGIN_SIM["longitude"],
            },
        ],
        remappings=[
            ("cmd_vel", "cmd_vel"),
            ("odom/local", "odom/local"),
            ("odom/global", "odom/global"),
            ("fromLL", "fromLL"),
        ],
    )

    return LaunchDescription(
        [
            simulation_arg,
            localization_simulator,
        ]
    )
