from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav_bringup.global_config import FRAMES


def generate_launch_description():
    sim_arg = DeclareLaunchArgument("sim", default_value="false", description="Run in simulation mode")
    sim = LaunchConfiguration("sim")

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
            localization_simulator,
        ]
    )
