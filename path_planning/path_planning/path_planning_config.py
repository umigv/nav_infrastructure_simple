from dataclasses import dataclass


@dataclass(frozen=True)
class PathPlanningConfig:
    """Configuration parameters for path planning.

    max_search_radius: Maximum distance (meters) from the robot's current position to search for a drivable point when
        the robot is in unknown space.
    interpolation_resolution: Distance (meters) between consecutive interpolated points when generating a path from the
        robot to the first drivable point.
    """

    frame_id: str = "odom"
    max_search_radius: float = 5.0
    interpolation_resolution: float = 0.05

    def __post_init__(self):
        if self.max_search_radius <= 0:
            raise ValueError("PathPlannerConfig: max_search_radius must be positive.")
        if self.interpolation_resolution <= 0:
            raise ValueError("PathPlannerConfig: interpolation_resolution must be positive.")
