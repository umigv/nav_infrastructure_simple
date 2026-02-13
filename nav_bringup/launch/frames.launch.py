from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.substitutions import FindPackageShare
import os
import yaml


def _load_frames(context, *args, **kwargs):
    share = FindPackageShare("nav_infrastructure_simple").perform(context)
    frames_path = os.path.join(share, "config", "frames.yaml")

    with open(frames_path, "r") as f:
        frames = yaml.safe_load(f) or {}

    defaults = {
        "base_frame": "base_link",
        "imu_frame": "imu_link",
        "gps_frame": "gps_link",
        "odom_frame": "odom",
        "map_frame": "map",
    }

    for k, v in defaults.items():
        context.launch_configurations[k] = str(frames.get(k, v))

    return []


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_load_frames),
    ])
