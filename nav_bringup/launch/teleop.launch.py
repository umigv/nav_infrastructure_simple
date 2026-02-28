from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav_bringup.global_config import FRAMES

CONTROLLERS = ("ps4", "xbox")


def launch_setup(context, *args, **kwargs) -> list[LaunchDescriptionEntity]:
    bringup_share = get_package_share_directory("nav_bringup")
    controller = LaunchConfiguration("controller").perform(context).strip().lower()
    teleop_params = PathJoinSubstitution([bringup_share, "config", f"teleop_{controller}.yaml"])

    return [
        Node(
            package="joy",
            executable="joy_node",
            name="joy",
            output="screen",
            parameters=[
                teleop_params,
            ],
        ),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy",
            output="screen",
            parameters=[
                teleop_params,
                {"frame": FRAMES["base_frame"]},
            ],
            remappings=[
                ("cmd_vel", "teleop_cmd_vel"),
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller",
                choices=list(CONTROLLERS),
                description="Controller profile",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
