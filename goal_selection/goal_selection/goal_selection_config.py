from dataclasses import dataclass


@dataclass(frozen=True)
class GoalSelectionConfig:
    goal_publish_period_seconds: float = 2.0
    waypoint_reached_threshold: float = 1.0
    waypoint_alignment_weight: float = 0.3
    waypoints_file_path: str
