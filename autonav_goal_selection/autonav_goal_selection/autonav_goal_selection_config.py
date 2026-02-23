from dataclasses import dataclass


@dataclass(frozen=True)
class GoalSelectionParams:
    """Parameters controlling weighting for goal selection in the occupancy grid.

    Attributes:
        lateral_quadratic_factor: Coefficient for quadratic penalty on lateral (y) distance from the robot's heading.
        behind_robot_linear_factor: Linear penalty scaling for cells behind the robot.
        behind_robot_penalty_distance_m: Distance (m) behind the robot up to which the linear penalty applies.
        waypoint_proximity_weight: Fixed score bonus for cells within waypoint_proximity_radius_m of the waypoint.
        waypoint_proximity_radius_m: Radius (m) around the waypoint that receives the proximity bonus.
        waypoint_dist_weight: A very small linear weight by distance to waypoint to prioritize staying on the side of
            the lane closer to the waypoint.
    """

    lateral_quadratic_factor: float = 0.25
    behind_robot_linear_factor: float = 1.0
    behind_robot_penalty_distance_m: float = 60.0
    waypoint_proximity_weight: float = 50.0
    waypoint_proximity_radius_m: float = 20.0
    waypoint_dist_weight: float = 0.01

    def __post_init__(self) -> None:
        if self.lateral_quadratic_factor < 0:
            raise ValueError("GoalSelectionParams: lateral_quadratic_factor must be >= 0")
        if self.behind_robot_linear_factor < 0:
            raise ValueError("GoalSelectionParams: behind_robot_linear_factor must be >= 0")
        if self.behind_robot_penalty_distance_m < 0:
            raise ValueError("GoalSelectionParams: behind_robot_penalty_distance_m must be >= 0")
        if self.waypoint_proximity_weight < 0:
            raise ValueError("GoalSelectionParams: waypoint_proximity_weight must be >= 0")
        if self.waypoint_proximity_radius_m < 0:
            raise ValueError("GoalSelectionParams: waypoint_proximity_radius_m must be >= 0")


@dataclass(frozen=True)
class AutonavGoalSelectionConfig:
    """Configuration for autonomous navigation goal selection.

    Attributes:
        goal_selection_params: Parameters for the goal selection algorithm.
        waypoints_file_path: Path to a JSON file containing the list of GPS waypoints to navigate.
        goal_publish_period_s: How often (seconds) to publish a new local goal.
        waypoint_reached_threshold: Distance (m) within which a waypoint is considered reached.
        map_frame_id: TF frame ID for the map coordinate frame.
        world_frame_id: TF frame ID for the world coordinate frame.
    """

    goal_selection_params: GoalSelectionParams
    waypoints_file_path: str
    goal_publish_period_s: float = 5.0
    waypoint_reached_threshold: float = 1.0
    map_frame_id: str = "map"
    world_frame_id: str = "odom"

    def __post_init__(self) -> None:
        if self.goal_publish_period_s <= 0:
            raise ValueError("AutonavGoalSelectionConfig: goal_publish_period_s must be > 0")
        if self.waypoint_reached_threshold <= 0:
            raise ValueError("AutonavGoalSelectionConfig: waypoint_reached_threshold must be > 0")
