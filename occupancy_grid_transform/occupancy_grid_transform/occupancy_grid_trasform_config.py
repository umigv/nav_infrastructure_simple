from dataclasses import dataclass

@dataclass(frozen=True)
class OccupancyGridTransformConfig:
    # Frame parameters
    grid_frame_id: str = "odom"
    robot_forward_offset_m: float = 0.60

    # Inflation parameters
    inflation_radius_cells: int = 10
    inflation_falloff_radius_cells: int = 20
    inflation_decay_factor: float = 0.9

    # Treat any cell > threshold as an obstacle source for inflation
    obstacle_threshold: int = 50
