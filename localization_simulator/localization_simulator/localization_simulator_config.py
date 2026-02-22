from dataclasses import dataclass


@dataclass(frozen=True)
class LocalizationSimulatorConfig:
    """Configuration for the localization simulator node.

    Attributes:
        gps_origin_latitude: GPS origin latitude in WGS84 degrees. All map-frame coordinates are relative to this point.
        gps_origin_longitude: GPS origin longitude in WGS84 degrees. All map-frame coordinates are relative to this point.
        update_period_s: Period of the position integration and TF publishing timer (s).
        cmd_vel_timeout_s: If no cmd_vel is received within this window the robot velocity is zeroed (s).
        map_frame_id: TF frame ID for the map frame.
        odom_frame_id: TF frame ID for the odometry frame.
        base_frame_id: TF frame ID for the robot base frame.
    """

    gps_origin_latitude: float
    gps_origin_longitude: float
    update_period_s: float = 0.01
    cmd_vel_timeout_s: float = 0.5
    map_frame_id: str = "map"
    odom_frame_id: str = "odom"
    base_frame_id: str = "base_link"

    def __post_init__(self) -> None:
        if self.update_period_s <= 0:
            raise ValueError("LocalizationSimulatorConfig: update_period_s must be > 0")
        if self.cmd_vel_timeout_s <= 0:
            raise ValueError("LocalizationSimulatorConfig: cmd_vel_timeout_s must be > 0")
