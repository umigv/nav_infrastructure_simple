from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav_bringup.global_config import FRAMES, GPS_ORIGIN_SIM


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("nav_bringup")
    localization_params = PathJoinSubstitution([bringup_share, "config", "localization.yaml"])
    simulation = LaunchConfiguration("simulation")
    use_enc_odom = LaunchConfiguration("use_enc_odom")

    localization_simulator = Node(
        package="localization_simulator",
        executable="localization_simulator",
        name="localization_simulator",
        condition=IfCondition(simulation),
        parameters=[
            {"map_frame_id": FRAMES["map_frame"]},
            {"odom_frame_id": FRAMES["odom_frame"]},
            {"base_frame_id": FRAMES["base_frame"]},
            {"gps_origin_latitude": GPS_ORIGIN_SIM["latitude"]},
            {"gps_origin_longitude": GPS_ORIGIN_SIM["longitude"]},
        ],
        remappings=[
            ("cmd_vel", "cmd_vel"),
            ("odom/local", "odom/local"),
            ("odom/global", "odom/global"),
            ("fromLL", "fromLL"),
            ("localization_initialized", "localization_initialized"),
        ],
    )

    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="screen",
        condition=IfCondition(AndSubstitution(NotSubstitution(simulation), NotSubstitution(use_enc_odom))),
        parameters=[
            localization_params,
            {"map_frame": FRAMES["map_frame"]},
            {"odom_frame": FRAMES["odom_frame"]},
            {"base_link_frame": FRAMES["base_frame"]},
            {"world_frame": FRAMES["odom_frame"]},
        ],
        remappings=[
            ("odometry/filtered", "odom/local"),
        ],
    )

    enc_odom = Node(
        package="localization",
        executable="enc_odom_publisher",
        name="enc_odom_publisher",
        output="screen",
        condition=IfCondition(AndSubstitution(NotSubstitution(simulation), use_enc_odom)),
        parameters=[
            {"odom_frame_id": FRAMES["odom_frame"]},
            {"base_frame_id": FRAMES["base_frame"]},
        ],
        remappings=[
            ("enc_vel", "enc_vel/raw"),
            ("odom", "odom/local"),
        ],
    )

    navsat = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        condition=UnlessCondition(simulation),
        parameters=[localization_params],
        remappings=[
            ("imu", "imu/raw"),
            ("gps/fix", "gps/raw"),
            ("odometry/filtered", "odom/local"),
            ("odometry/gps", "odom/gps"),
            ("datum", "datum"),
        ],
    )

    ekf_global = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global",
        output="screen",
        condition=UnlessCondition(simulation),
        parameters=[
            localization_params,
            {"map_frame": FRAMES["map_frame"]},
            {"odom_frame": FRAMES["odom_frame"]},
            {"base_link_frame": FRAMES["base_frame"]},
            {"world_frame": FRAMES["map_frame"]},
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
        condition=UnlessCondition(simulation),
        remappings=[
            ("gps", "gps/raw"),
            ("set_datum", "datum"),
            ("localization_initialized", "localization_initialized"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "simulation",
                default_value="false",
                description="Run in simulation mode",
            ),
            DeclareLaunchArgument(
                "use_enc_odom",
                default_value="false",
                description="Replace ekf_local with encoder odometry integration",
            ),
            localization_simulator,
            ekf_local,
            enc_odom,
            navsat,
            ekf_global,
            gps_origin_initializer,
        ]
    )
