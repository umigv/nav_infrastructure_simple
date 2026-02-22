from dataclasses import dataclass


@dataclass(frozen=True)
class PathTrackingConfig:
    """Configuration for path tracking.

    Attributes:
        max_angular_speed_radps: Maximum angular velocity command (rad/s).
            Commands are scaled down to stay within this limit.
        base_lookahead_distance_m: Lookahead distance when stationary (m). Added to the speed-dependent term.
        min_lookahead_distance_m: Minimum clamped lookahead distance (m).
        max_lookahead_distance_m: Maximum clamped lookahead distance (m).
        lookahead_speed_gain: Gain applied to current speed when computing adaptive lookahead distance.
        linear_speed_gain: Gain applied to lookahead distance to produce the linear velocity command.
        control_period_s: Period of the control loop timer (s).
        base_frame_id: Frame ID of the robot base, used as the child frame in odometry validation.
        odom_frame_id: Frame ID of the odometry frame, used to validate incoming odom and path messages.
    """

    max_angular_speed_radps: float = 0.6
    base_lookahead_distance_m: float = 0.1
    min_lookahead_distance_m: float = 0.1
    max_lookahead_distance_m: float = 0.4
    lookahead_speed_gain: float = 0.55
    linear_speed_gain: float = 2.0
    control_period_s: float = 0.1
    base_frame_id: str = "base_link"
    odom_frame_id: str = "odom"

    def __post_init__(self) -> None:
        if self.max_angular_speed_radps <= 0:
            raise ValueError("PathTrackingConfig: max_angular_speed_radps must be > 0")
        if self.min_lookahead_distance_m <= 0:
            raise ValueError("PathTrackingConfig: min_lookahead_distance_m must be > 0")
        if self.max_lookahead_distance_m <= 0:
            raise ValueError("PathTrackingConfig: max_lookahead_distance_m must be > 0")
        if self.min_lookahead_distance_m > self.max_lookahead_distance_m:
            raise ValueError("PathTrackingConfig: min_lookahead_distance_m must be <= max_lookahead_distance_m")
        if self.lookahead_speed_gain <= 0:
            raise ValueError("PathTrackingConfig: lookahead_speed_gain must be > 0")
        if self.linear_speed_gain <= 0:
            raise ValueError("PathTrackingConfig: linear_speed_gain must be > 0")
        if self.control_period_s <= 0:
            raise ValueError("PathTrackingConfig: control_period_s must be > 0")
