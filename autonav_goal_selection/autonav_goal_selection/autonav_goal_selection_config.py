from dataclasses import dataclass


@dataclass(frozen=True)
class GoalSelectionParams:
    """Parameters for the goal selection algorithm."""


@dataclass(frozen=True)
class AutonavGoalSelectionConfig:
    """Configuration for autonomous navigation goal selection.

    map_frame_id: TF frame ID for the map coordinate frame.
    world_frame_id: TF frame ID for the world coordinate frame.
    goal_publish_period_s: How often (seconds) to publish a new goal.
    waypoints_file_path: Path to a file containing the list of waypoints to navigate.
    goal_selection_params: Parameters for the goal selection algorithm.
    waypoint_reached_threshold: Distance (meters) within which a waypoint is considered reached.
    """

    waypoints_file_path: str
    goal_selection_params: GoalSelectionParams
    map_frame_id: str = "map"
    world_frame_id: str = "world"
    goal_publish_period_s: float = 2.0
    waypoint_reached_threshold: float = 1.0

    def __post_init__(self):
        if self.goal_publish_period_s <= 0:
            raise ValueError("AutonavGoalSelectionConfig: goal_publish_period_s must be positive")
        if self.waypoint_reached_threshold <= 0:
            raise ValueError("AutonavGoalSelectionConfig: waypoint_reached_threshold must be positive")
