from launch import LaunchDescription
from launch_ros.actions import Node

# [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]

def generate_launch_description():
    twist_data_topic = "/enc_vel" # TODO: tune encoder covariance if needed
    imu_data_topic = "/zed/zed_node/imu/data"
    gps_data_topic = "/gps/raw"

    odom_frame = "odom"
    map_frame = "map"
    base_frame = "base_link"
    imu_frame = "zed_imu_link"
    gps_frame = "gps_link"

    gps = Node(
        package="gps_publisher",
        executable="gps_publisher",
        name="gps_publisher",
        output="screen",
        remappings=[
            ("gps", gps_data_topic),
        ]
    )

    tf_base_to_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_imu",
        output="screen",
        arguments=[
            "--x", "-0.0508", "--y", "0.0", "--z", "1.524",
            "--roll", "0.0", "--pitch", "-0.488692", "--yaw", "0.0",
            "--frame-id", base_frame,
            "--child-frame-id", imu_frame,
        ],
    )

    tf_base_to_gps = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_gps",
        output="screen",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.8128",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", base_frame,
            "--child-frame-id", gps_frame,
        ],
    )

    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="screen",
        parameters=[{
            "frequency": 50.0,
            "sensor_timeout": 0.1,
            "two_d_mode": True,
            "print_diagnostics": True,

            "map_frame": map_frame,
            "odom_frame": odom_frame,
            "base_link_frame": base_frame,
            "world_frame": odom_frame,
            "publish_tf": True,

            "twist0": twist_data_topic,
            "twist0_config": [
                False,  False,  False,
                False, False, False,
                True,  False,  False,
                False, False, True,
                False, False, False
            ],
            "twist0_queue_size": 50,
            "twist0_nodelay": True,
            "twist0_differential": False,
            "twist0_relative": False,

            "imu0": imu_data_topic,
            "imu0_config": [
                False, False, False,
                False,  False, False,
                False, False, False,
                False,  False, True,
                False, False, False
            ],
            "imu0_queue_size": 100,
            "imu0_differential": False,
            "imu0_relative": False,
        }],
        remappings=[
            ("odometry/filtered", "/odometry/local"),
        ],
    )

    navsat = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[{
            "frequency": 20.0,

            "map_frame": map_frame,
            "odom_frame": odom_frame,
            "base_link_frame": base_frame,
            "world_frame": odom_frame,
            "imu_frame": imu_frame,

            "wait_for_datum": True,
            "zero_altitude": True,
            "broadcast_utm_transform": False,
            "publish_filtered_gps": True,
            "use_odometry_yaw": True,
            "yaw_offset": 0.0,
            "magnetic_declination_radians": 0.0,
        }],
        remappings=[
            ("imu", imu_data_topic),
            ("gps/fix", gps_data_topic),
            ("odometry/filtered", "/odometry/local"),
            ("odometry/gps", "/odometry/gps"),
            ("gps/filtered", "/gps"),
        ],
    )

    ekf_global = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global",
        output="screen",
        parameters=[{
            "frequency": 30.0,
            "sensor_timeout": 1.5,
            "two_d_mode": True,
            "print_diagnostics": True,

            "map_frame": map_frame,
            "odom_frame": odom_frame,
            "base_link_frame": base_frame,
            "world_frame": map_frame,
            "publish_tf": True,

            "odom0": "/odometry/local",
            "odom0_config": [
                True,  True,  False,
                False, False, True,
                True,  False,  False,
                False, False, False,
                False, False, False
            ],
            "odom0_queue_size": 20,

            "odom1": "/odometry/gps",
            "odom1_config": [
                True,  True,  False,
                False, False, False,
                False, False, False,
                False, False, False,
                False, False, False
            ],
            "odom1_queue_size": 20,
            "odom1_differential": False,
            "odom1_relative": False,
        }],
        remappings=[
            ("odometry/filtered", "/odometry/global"),
        ],
    )
    
    gps_origin_initializer = Node(
        package="localization",
        executable="gps_origin_initializer",
        name="gps_origin_initializer",
        output="screen",
        remappings=[
            ("gps", gps_data_topic),
        ],
    )

    return LaunchDescription([
        gps,
        tf_base_to_imu,
        tf_base_to_gps,
        ekf_local,
        navsat,
        ekf_global,
        gps_origin_initializer,
    ])
