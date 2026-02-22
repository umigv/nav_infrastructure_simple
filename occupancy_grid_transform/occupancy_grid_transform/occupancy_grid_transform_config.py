from dataclasses import dataclass


@dataclass(frozen=True)
class InflationParams:
    """Parameters controlling obstacle inflation for an occupancy grid.

    Inflation is performed in grid (cell) space and consists of a fully inflated core region surrounded by an optional
    falloff region where obstacle influence decays with distance.

    Attributes:
        inflation_radius_cells: Radius (in grid cells) of the fully inflated obstacle core. All cells within this
            distance of an occupied cell are treated as maximally inflated, preventing planners from routing paths
            too close to obstacles.
        inflation_falloff_radius_cells: Maximum radius (in grid cells) of obstacle influence. Beyond
            `inflation_radius_cells`, obstacle influence decays until this radius is reached. Cells outside this
            radius are unaffected by inflation.
        inflation_decay_factor: Decay factor applied to obstacle influence in the falloff region. Values closer to
            1.0 result in slower decay and more conservative paths; values closer to 0.0 decay more aggressively.
    """

    inflation_radius_cells: int = 10
    inflation_falloff_radius_cells: int = 20
    inflation_decay_factor: float = 0.9

    def __post_init__(self) -> None:
        if self.inflation_radius_cells < 0:
            raise ValueError("InflationParams: inflation_radius_cells must be >= 0")
        if self.inflation_falloff_radius_cells < 0:
            raise ValueError("InflationParams: inflation_falloff_radius_cells must be >= 0")
        if self.inflation_radius_cells > self.inflation_falloff_radius_cells:
            raise ValueError("InflationParams: inflation_falloff_radius_cells must be >= inflation_radius_cells")
        if not (0 < self.inflation_decay_factor < 1):
            raise ValueError("InflationParams: inflation_decay_factor must be between 0 and 1")


@dataclass(frozen=True)
class OccupancyGridTransformConfig:
    """Configuration for transforming a robot-centric occupancy grid into a world-aligned ROS `nav_msgs/msg/OccupancyGrid`.

    Attributes:
        inflation_params: Parameters controlling obstacle inflation applied to the grid prior to publishing.
        frame_id: TF frame ID in which the transformed occupancy grid is published. All grid coordinates and the
            computed origin pose are expressed in this frame.
    """

    inflation_params: InflationParams
    frame_id: str = "odom"
