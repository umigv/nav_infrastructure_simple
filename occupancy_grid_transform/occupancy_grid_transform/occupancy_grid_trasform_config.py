from dataclasses import dataclass


@dataclass(frozen=True)
class InflationParams:
    """
    Parameters controlling obstacle inflation for an occupancy grid.

    These parameters define how occupied cells are expanded to create a safety buffer around obstacles before planning.
    Inflation is performed in grid (cell) space and consists of a fully inflated core region surrounded by an optional
    falloff region where obstacle influence decays with distance.
    """

    """Radius (in grid cells) of the fully inflated obstacle core.

    All cells within this distance of an occupied cell are treated as
    maximally inflated, preventing planners from routing paths too close
    to obstacles.
    """
    inflation_radius_cells: int = 10

    """Maximum radius (in grid cells) of obstacle influence.

    Beyond `inflation_radius_cells`, obstacle influence decays until this
    radius is reached. Cells outside this radius are unaffected by
    inflation.
    """
    inflation_falloff_radius_cells: int = 20

    """Decay factor applied to obstacle influence in the falloff region.

    Controls how quickly inflation strength decreases with distance from
    the obstacle. Values closer to 1.0 result in slower decay and more
    conservative paths; values closer to 0.0 decay more aggressively.
    """
    inflation_decay_factor: float = 0.9

    def __post_init__(self):
        if self.inflation_radius_cells < 0:
            raise ValueError("InflationParams: inflation_radius_cells must be > 0")

        if self.inflation_falloff_radius_cells < 0:
            raise ValueError("InflationParams: inflation_falloff_radius_cells must be > 0")

        if self.inflation_radius_cells > self.inflation_falloff_radius_cells:
            raise ValueError("InflationParams: inflation_falloff_radius_cells must be >= inflation_radius_cells")

        if not (0 < self.inflation_decay_factor < 1):
            raise ValueError("InflationParams: inflation_decay_factor must be between 0 and 1")


@dataclass(frozen=True)
class WeightParams:
    """
    Parameters controlling weighting for inflated occupancy grid.
    """

    """
    Quadratic Factor: coefficient determining how strongly the y direction is are weighted as they move away from the center of the grid.
    """
    quadratic_factor: float = 0.25

    """
    linear factor: how the weighting scales with distance from y=0 (behind the robot)
    """
    linear_factor: float = 1

    """
    Linear ratio: the percent of the grid that is scaled by linear weight
    """

    linear_ratio:  float = .75

    """
    Top bar size: number of rows discouraged from being driveable at the opposite end of the grid from the robot.
    """

    top_bar_size: int = 30

    """
    Top bar weight: extent to which the rows at the opposite end of the grid from the robot are weighted.
    """

    top_bar_weight: int = 15
    

    def __post_init__(self):
        if self.linear_ratio >  1 or self.linear_ratio <  0:
            raise ValueError("WeightParams: linear_ratio must be between 0 and 1.")


@dataclass(frozen=True)
class OccupancyGridTransformConfig:
    """
    Configuration for transforming a robot-centric occupancy grid into a world-aligned ROS `nav_msgs/msg/OccupancyGrid`.

    This configuration controls obstacle inflation, grid anchoring relative to the robot pose, and the coordinate frame
    in which the transformed grid is published.
    """

    """Parameters controlling obstacle inflation applied to the grid prior to publishing."""
    inflation_params: InflationParams

    """Parameters controlling positional weighting applied to the grid prior to publishing."""
    weight_params: WeightParams

    """TF frame ID in which the transformed occupancy grid is published.

    All grid coordinates and the computed origin pose are expressed in
    this frame.
    """
    grid_frame_id: str = "odom"

    """Forward offset (in meters) from the robot to the near edge of the grid.

    Used when computing the grid origin. The grid is assumed to be centered laterally on the robot (+y) and to begin 
    `robot_forward_offset_m` meters in front of the robot along +x.
    """
    robot_forward_offset_m: float = 0.60

