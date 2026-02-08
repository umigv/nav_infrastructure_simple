from __future__ import annotations

import math
from itertools import product

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import OccupancyGrid
from nav_utils.geometry import get_yaw_radians_from_quaternion, rotate_by_yaw

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


def compute_origin_pose(
    odom: Pose | None, robot_forward_offset_m: float, grid_height_cells: int, resolution: float
) -> Pose:
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
            position=Point(x=odom.position.x + rotated.x, y=odom.position.y + rotated.y, z=0.0),
            orientation=odom.orientation,
        )
    else:
        return Pose(
            position = Point(x=local.x, y=local.y, z=0.0),
            orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        )
    
def weight_grid(
        grid: np.ndarray,
        quadratic_factor: float = .25,
        linear_factor: float = 1,
        linear_ratio:  float = .75,
        top_bar_size: int = 30,
        top_bar_weight: int = 15
    ) -> np.ndarray:
    """Generates the weighting grid of an occupancy grid of a given size as a 2D Numpy Array. Will need to play with default weightings"""


    #hopefully this doesn't cause pointer weirdness
    width, height = grid.shape 

    #see if these need to be changed
    x = 0
    while (x <= width/2): 
        y = 0
        while (y < height): 
               #weight the bottom. this is weighted assuming the top is 0.
                if (y < (height - (height* linear_ratio))):
                    grid[x,y] += (height*linear_ratio-(y+1)) *  linear_factor
                #weight the top bar a little. this is weighted assuming the top is 0.
                if (y >= height - top_bar_size):
                    grid[x,y] += top_bar_weight
                #quadratic rating on the center
                #change the weighting as needed
                grid[x,y] += quadratic_factor * pow(float(width/2) - float(x), 2)

                #set this to max if it's greater
                grid[x,y] = min(grid[x,y], 100)
                grid[width-1-x,y] = grid[x,y]

                #I've just thought of something. Last year we used a matrix to store the costs. This year we're just using inflation grids.
                y+=1
        x+=1

    return grid
