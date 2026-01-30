from dataclasses import dataclass

@dataclass(frozen=True)
class InflationParams:
    inflation_radius_cells: int = 10
    inflation_falloff_radius_cells: int = 20
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
class OccupancyGridTransformConfig:
    inflation_params: InflationParams
    grid_frame_id: str = "odom"
    robot_forward_offset_m: float = 0.60
