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
    """
    Convert an incoming CV-frame occupancy grid into a ROS-aligned 2D array.

    The CV-frame is expected to have the following conventions
    - Column major
    - Height = number of cells in +x direction, width = number of cells in +y direction
    - Top left is origin

    The transformed 2D array has the standard ROS2 convention, where
    - Row major
    - Height = number of cells in +y direction, width = number of cells in +x direction
    - Bottom left is origin


    For example, after transform, with a 3 wide 2 tall grid:

                   4 5 
            R -->  2 3
                   0 1

        ^
        |
        y 
          x-->

    R is the robot position, and the numbers represent the index of each cell in the incoming grid data array.
    The grid is indexed with (y, x) in 2d, and (y * width + x) in 1d.


    Args:
        grid: Incoming `nav_msgs/msg/OccupancyGrid`.

    Returns:
        A 2D `np.ndarray` of dtype `int8` with shape `(height, width)` under ROS convention.
    """

    grid = np.asarray(grid.data, dtype=np.int8).reshape((grid.info.width, grid.info.height), order='F')
    return np.flipud(grid)

def inflate_grid(grid: np.ndarray, params: InflationParams) -> np.ndarray:
    """
    Inflate obstacles in a 2D occupancy grid.

    For each occupied cell (value 100), this expands obstacle values based on the provided InflationParams

    Args:
        grid: 2D occupancy grid of shape `(height, width)` with values in `[-1, 100]` (unknown/free/occupied).
        params: Inflation tuning parameters (hard radius, soft radius, decay).

    Returns:
        A new 2D `np.ndarray` (dtype `int8`) of the same shape containing the inflated occupancy values.
    """

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
    """
    Compute the published OccupancyGrid origin pose.

    The returned pose corresponds to `OccupancyGrid.info.origin`: the world pose of the grid's (0, 0) cell (i.e., the 
    grid's lower-left corner in the grid's own coordinate system).

    This transform assumes the grid is:
    - shifted `robot_forward_offset_m` meters forward of the robot along +x
    - centered laterally on the robot, so the origin is `height/2` cells to the right (negative y) of the robot

    Args:
        odom: Robot pose in the target frame. If None, the origin is assumed to be (0, 0, 0).
        robot_forward_offset_m: Forward distance (meters) from the robot to the grid origin along +x.
        grid_height_cells: Grid height (cells), used to center the grid about the robot in y.
        resolution: Cell size (meters/cell).

    Returns:
        A `geometry_msgs/msg/Pose` suitable for `OccupancyGrid.info.origin`.
    """

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
