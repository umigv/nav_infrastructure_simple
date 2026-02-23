import math
from itertools import product

import numpy as np

from .occupancy_grid_transform_config import InflationParams


def add_border(grid: np.ndarray) -> np.ndarray:
    """
    Add a 1-cell occupied border on the top, bottom, and right edges of the grid.

    Sets the top row, bottom row, and right column to 100 (occupied). The left column (y=0, closest
    to the robot) is left untouched. When followed by `inflate_grid`, the border produces a soft
    falloff that discourages planning near grid edges.

    Args:
        grid: 2D occupancy grid of shape `(height, width)`.

    Returns:
        The same grid array, modified in place.
    """
    grid[-1, :] = 100  # top row
    grid[0, :] = 100  # bottom row
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
