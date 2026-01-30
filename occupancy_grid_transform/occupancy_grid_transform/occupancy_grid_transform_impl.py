from __future__ import annotations

from nav_msgs.msg import OccupancyGrid
from typing import Optional
import math
import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_utils.geometry import rotate_by_yaw
from .occupancy_grid_trasform_config import InflationParams
from itertools import product
from nav_utils.geometry import get_yaw_radians_from_quaternion, rotate_by_yaw

def cv_occupancy_grid_to_ros_grid(grid: OccupancyGrid) -> np.ndarray:
    grid = np.asarray(grid.data, dtype=np.int8).reshape((grid.info.width, grid.info.height), order='F')
    return np.flipud(grid)

def inflate_grid(grid: np.ndarray, params: InflationParams) -> np.ndarray:
    r_hard = params.inflation_radius_cells
    r_soft = params.inflation_falloff_radius_cells
    decay  = params.inflation_decay_factor
    height, width = grid.shape 

    output = grid.astype(np.int8, copy=True)

    def inflate(x: int, y: int) -> None:
        for dx, dy in product(range(-r_soft, r_soft + 1), repeat=2):
            dist = math.hypot(dx, dy)
            if dist > r_soft:
                continue

            nx, ny = x + dx, y + dy
            if not (0 <= nx < width) or not (0 <= ny < height):
                continue

            value = int(100 if dist <= r_hard else round(100 * (decay ** (dist - r_hard))))
            output[ny, nx] = max(output[ny, nx], value)

    ys, xs = np.where(grid == 100)
    for x, y in zip(xs.tolist(), ys.tolist()):
        inflate(x, y)

    return output.astype(np.int8)

def compute_origin_pose(odom: Optional[Pose], robot_forward_offset_m: float, grid_height_cells: int, resolution: float) -> Pose:
    local = Point(x=robot_forward_offset_m, y=-grid_height_cells * resolution / 2.0, z=0.0)

    if odom is not None:
        yaw = get_yaw_radians_from_quaternion(odom.orientation)
        rotated = rotate_by_yaw(local, yaw)

        return Pose(
            position = Point(x=odom.position.x + rotated.x, y=odom.position.y+rotated.y, z=0.0),
            orientation = odom.orientation
        )
    else:
        return Pose(
            position = Point(x=local.x, y=local.y, z=0.0),
            orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        )
