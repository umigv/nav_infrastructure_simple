from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ===== Frames =====
    base_link = "base_link"
    odom = "odom"
    map_frame = "map"

    # ===== Topics (edit to match yours) =====
    wheel_odom_topic = "/wheel/odom"     # nav_msgs/Odometry (your hand calc)
    imu_topic = "/imu/data"              # sensor_msgs/Imu (ZED2i)
    gps_fix_topic = "/gps/fix"           # sensor_msgs/NavSatFix

    odom_filtered_topic = "/odometry/filtered"  # local EKF output
    gps_odom_topic = "/odometry/gps"            # navsat output (GPS in local XY as Odometry)
    odom_global_topic = "/odometry/global"      # global EKF output (optional)

    # ========== Local EKF: wheel + IMU -> odom->base_link ==========
    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="screen",
        parameters=[{
            "frequency": 50.0,
            "sensor_timeout": 0.1,
            "two_d_mode": True,

            "map_frame": map_frame,
            "odom_frame": odom,
            "base_link_frame": base_link,

            # world_frame determines which transform this EKF publishes:
            # world_frame = odom => publishes odom -> base_link
            "world_frame": odom,

            "publish_tf": True,

            # Wheel odom
            "odom0": wheel_odom_topic,
            # [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
            "odom0_config": [
                True,  True,  False,
                False, False, True,
                True,  True,  False,
                False, False, True,
                False, False, False
            ],
            "odom0_queue_size": 10,
            "odom0_differential": False,
            "odom0_relative": False,

            # IMU (ZED2i)
            "imu0": imu_topic,
            "imu0_config": [
                False, False, False,
                True,  True,  True,     # use roll/pitch/yaw orientation if present
                False, False, False,
                True,  True,  True,     # use angular velocity
                False, False, False
            ],
            "imu0_queue_size": 50,
            "imu0_differential": False,
            "imu0_relative": True,      # typical if IMU yaw is not absolute to true north
            "imu0_remove_gravitational_acceleration": True,
        }],
        remappings=[
            ("odometry/filtered", odom_filtered_topic),
        ],
    )

    # ========== navsat_transform: GPS -> local XY Odometry ==========
    navsat = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[{
            "frequency": 20.0,

            # We'll set datum ourselves (avg of first N GPS samples)
            "wait_for_datum": True,

            "zero_altitude": True,
            "broadcast_utm_transform": False,
            "publish_filtered_gps": True,

            # Yaw source for aligning GPS ENU frame.
            # If IMU yaw is “good enough”, this is fine. Otherwise tune yaw_offset/declination.
            "use_odometry_yaw": True,

            "yaw_offset": 0.0,
            "magnetic_declination_radians": 0.0,
        }],
        remappings=[
            ("imu", imu_topic),
            ("gps/fix", gps_fix_topic),
            ("odometry/filtered", odom_filtered_topic),
            ("odometry/gps", gps_odom_topic),
            ("gps/filtered", "/gps/filtered"),
        ],
    )

    # ========== Global EKF: local odom + GPS odom -> map->odom ==========
    ekf_global = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global",
        output="screen",
        parameters=[{
            "frequency": 30.0,
            "sensor_timeout": 0.2,
            "two_d_mode": True,

            "map_frame": map_frame,
            "odom_frame": odom,
            "base_link_frame": base_link,

            # world_frame = map => publishes map -> odom
            "world_frame": map_frame,

            "publish_tf": True,

            # Smooth local motion model
            "odom0": odom_filtered_topic,
            "odom0_config": [
                True,  True,  False,
                False, False, True,
                True,  True,  False,
                False, False, True,
                False, False, False
            ],
            "odom0_queue_size": 10,

            # GPS as local XY (from navsat_transform)
            "odom1": gps_odom_topic,
            "odom1_config": [
                True,  True,  False,
                False, False, False,
                False, False, False,
                False, False, False,
                False, False, False
            ],
            "odom1_queue_size": 10,
            "odom1_differential": False,
            "odom1_relative": False,
        }],
        remappings=[
            ("odometry/filtered", odom_global_topic),
        ],
    )

    # ========== Static TF publisher (base_link -> imu_link/gps_link) ==========
    static_frames = Node(
        package="my_localization",
        executable="static_sensor_frames",
        name="static_sensor_frames",
        output="screen",
        parameters=[{
            "base_frame": base_link,

            # edit these to match your sensor frame_ids
            "imu_frame": "imu_link",
            "gps_frame": "gps_link",

            # XYZ in meters relative to base_link
            "imu_xyz": [0.10, 0.00, 0.20],
            "gps_xyz": [0.25, 0.00, 0.30],

            # RPY in radians relative to base_link (use if sensors are mounted rotated)
            "imu_rpy": [0.0, 0.0, 0.0],
            "gps_rpy": [0.0, 0.0, 0.0],
        }],
    )

    # ========== Datum initializer (avg first 10 GPS fixes) ==========
    set_datum_avg = Node(
        package="my_localization",
        executable="set_datum_avg",
        name="set_datum_avg",
        output="screen",
        parameters=[{
            "gps_fix_topic": gps_fix_topic,
            "set_datum_service": "/navsat_transform/set_datum",
            "num_samples": 10,
            "min_status": 0,
        }],
    )

    return LaunchDescription([
        static_frames,
        ekf_local,
        navsat,
        ekf_global,
        set_datum_avg,
    ])
