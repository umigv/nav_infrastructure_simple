from dataclasses import dataclass


@dataclass(frozen=True)
class PathPlannerConfig:
    max_search_radius: float = 5.0
    interpolation_resolution: float = 0.05
