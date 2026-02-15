from dataclasses import dataclass


@dataclass(frozen=True)
class LocalizationSimulatorConfig:
    """Configuration for the localization simulator node.

    datum_latitude / longitude: GPS origin in WGS84 degrees. All map-frame coordinates are relative to this datum.
    update_period_s: how often the position integration + TF publishing runs.
    cmd_vel_timeout_s: if no cmd_vel is received within this window the robot velocity is zeroed.
    map_odom_noise_stddev: standard deviation of random offset applied to the map->odom transform each update.
        Set to 0.0 for identity transform.
    """

    map_frame_id: str = "map"
    odom_frame_id: str = "odom"
    base_frame_id: str = "base_link"
    datum_latitude: float = 42.29436962024459
    datum_longitude: float = -83.70836182187283
    update_period_s: float = 0.01
    cmd_vel_timeout_s: float = 0.5

    def __post_init__(self) -> None:
        if self.update_period_s <= 0:
            raise ValueError("LocalizationSimulatorConfig: update_period_s must be > 0")
        if self.cmd_vel_timeout_s <= 0:
            raise ValueError("LocalizationSimulatorConfig: cmd_vel_timeout_s must be > 0")
