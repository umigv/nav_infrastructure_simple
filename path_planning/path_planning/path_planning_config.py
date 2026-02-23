from dataclasses import dataclass


@dataclass(frozen=True)
class PathPlanningConfig:
    """Configuration parameters for path planning.

    Attributes:
        max_search_radius_m: Maximum distance (m) from the robot to search for a drivable point when the robot is in
            unknown space.
        interpolation_resolution_m: Distance (m) between consecutive interpolated points when bridging from the robot
            to the first drivable point.
        spline_smoothing: Smoothing factor passed to scipy splprep. Higher values smooth more aggressively.
        frame_id: TF frame ID for the occupancy grid, odometry, goal, and published path.
    """

    max_search_radius_m: float = 5.0
    interpolation_resolution_m: float = 0.05
    spline_smoothing: float = 0.1
    frame_id: str = "odom"

    def __post_init__(self) -> None:
        if self.max_search_radius_m <= 0:
            raise ValueError("PathPlanningConfig: max_search_radius_m must be > 0.")
        if self.interpolation_resolution_m <= 0:
            raise ValueError("PathPlanningConfig: interpolation_resolution_m must be > 0.")
        if self.spline_smoothing < 0:
            raise ValueError("PathPlanningConfig: spline_smoothing must be >= 0.")
