from dataclasses import dataclass

@dataclass(frozen=True)
class InflationParams:
    inflation_radius_cells: int = 10
    inflation_falloff_radius_cells: int = 20
    inflation_decay_factor: float = 0.9

@dataclass(frozen=True)
class OccupancyGridTransformConfig:
    inflation_params: InflationParams
    grid_frame_id: str = "odom"
    robot_forward_offset_m: float = 0.60
