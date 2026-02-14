from __future__ import annotations

import math
from itertools import product

import numpy as np
from nav_msgs.msg import OccupancyGrid

from .occupancy_grid_trasform_config import InflationParams


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

        1d indexing by (y * width + x):

               4 5
        R -->  2 3
               0 1


        2d indexing by (y, x):

              (2, 0) (2, 1)
        R --> (1, 0) (1, 1)
              (0, 0) (0, 1)

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

    grid = np.asarray(grid.data, dtype=np.int8).reshape((grid.info.width, grid.info.height), order="F")
    return np.flipud(grid)


def add_border(grid: np.ndarray) -> np.ndarray:
    """
    Add a 1-cell occupied border on all edges except the bottom (closest to robot).

    Sets the top row, left column, and right column to 100 (occupied). The bottom row (y=0) is
    left untouched since it is nearest to the robot. When followed by `inflate_grid`, the border
    produces a soft falloff that discourages planning near grid edges.

    Args:
        grid: 2D occupancy grid of shape `(height, width)`.

    Returns:
        The same grid array, modified in place.
    """
    grid[-1, :] = 100  # top row
    grid[:, 0] = 100  # left column
    grid[:, -1] = 100  # right column
    return grid


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
    decay = params.inflation_decay_factor
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
    for x, y in zip(xs.tolist(), ys.tolist(), strict=True):
        inflate(x, y)

    return output.astype(np.int8)
