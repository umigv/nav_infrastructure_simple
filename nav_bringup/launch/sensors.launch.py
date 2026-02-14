from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from nav_bringup.global_config import FRAMES

def generate_launch_description():
    bringup_share = FindPackageShare("nav_bringup")
    imu_params = PathJoinSubstitution(
        [bringup_share, "config", "imu.yaml"]
    )
    gps_params = PathJoinSubstitution(
        [bringup_share, "config", "gps.yaml"]
    )

    base_frame = FRAMES["base_frame"]
    imu_frame  = FRAMES["imu_frame"]
    gps_frame  = FRAMES["gps_frame"]
    
    vectornav_driver = Node(
        package="vectornav",
        executable="vectornav",
        name="vectornav",
        output="screen",
        parameters=[imu_params],
    )

    vectornav_msgs = Node(
        package="vectornav",
        executable="vn_sensor_msgs",
        name="vn_sensor_msgs",
        output="screen",
        parameters=[imu_params],
    )

    gps = Node(
        package="gps_publisher",
        executable="gps_publisher",
        name="gps_publisher",
        output="screen",
        parameters=[gps_params],
        remappings=[
            ("gps", "gps/raw"),
        ],
    )

    tf_base_to_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_imu",
        output="screen",
        arguments=[
            "--x",
            "-0.0508",
            "--y",
            "0.0",
            "--z",
            "1.524",
            "--roll",
            "0.0",
            "--pitch",
            "-0.488692",
            "--yaw",
            "0.0",
            "--frame-id",
            base_frame,
            "--child-frame-id",
            imu_frame,
        ],
    )

    tf_base_to_gps = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_gps",
        output="screen",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.8128",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            base_frame,
            "--child-frame-id",
            gps_frame,
        ],
    )


    return LaunchDescription(
        [
            vectornav_driver,
            vectornav_msgs,
            gps,
            tf_base_to_imu,
            tf_base_to_gps,
        ]
    )
