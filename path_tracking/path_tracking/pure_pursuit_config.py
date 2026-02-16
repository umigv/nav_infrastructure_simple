from dataclasses import dataclass


@dataclass(frozen=True)
class PurePursuitConfig:
    max_angular_speed: float = 0.6
    base_lookahead: float = 0.1  # base lookahead distance (m)
    min_lookahead: float = 0.1  # minimum lookahead distance (m)
    max_lookahead: float = 0.4  # maximum lookahead distance (m)
    k_speed: float = 0.55  # scaling factor for current speed when calculating lookahead distance
    speed_percent: float = 2.0  # for scaling linear velocity
    smoothing: float = 0.1  # smoothing factor for spline
    control_loop_sample_time: float = 0.1  # period of control loop timer (s)

    def __post_init__(self):
        if self.max_angular_speed < 0:
            raise ValueError("PurePursuitParams: max_angular_speed must be > 0")

        if self.base_lookahead < 0:
            raise ValueError("PurePursuitParams: base_lookahead must be > 0")

        if self.min_lookahead < 0:
            raise ValueError("PurePursuitParams: min_lookahead must be > 0")

        if self.max_lookahead < 0:
            raise ValueError("PurePursuitParams: max_lookahead must be > 0")

        if self.min_lookahead > self.max_lookahead:
            raise ValueError("PurePursuitParams: min_lookahead must be <= max_lookahead")

        if self.k_speed < 0:
            raise ValueError("PurePursuitParams: k_speed must be > 0")

        if self.speed_percent < 0:
            raise ValueError("PurePursuitParams: speed_percent must be > 0")

        if self.smoothing < 0:
            raise ValueError("PurePursuitParams: smoothing must be > 0")

        if self.control_loop_sample_time < 0:
            raise ValueError("PurePursuitParams: control_loop_sample_time must be > 0")
