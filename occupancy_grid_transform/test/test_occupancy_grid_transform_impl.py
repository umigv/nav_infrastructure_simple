import numpy as np
from nav_msgs.msg import OccupancyGrid
from occupancy_grid_transform.occupancy_grid_transform_impl import (
    add_border,
    cv_occupancy_grid_to_ros_grid,
    inflate_grid,
)
from occupancy_grid_transform.occupancy_grid_trasform_config import InflationParams


def test_cv_occupancy_grid_to_ros_grid():
    # Provider example:
    # data=[1,2,3,4,5,6], width=3, height=2 corresponds to:
    # 1 4
    # 2 5
    # 3 6
    #
    # Expected ROS Convention should be [3, 6, 2, 5, 1, 4] where grid[y, x] gives the value
    msg = OccupancyGrid()
    msg.info.width = 3
    msg.info.height = 2
    msg.data = [1, 2, 3, 4, 5, 6]

    grid = cv_occupancy_grid_to_ros_grid(msg)

    assert grid.shape == (3, 2)
    assert grid.dtype == np.int8
    assert grid.tolist() == [
        [3, 6],
        [2, 5],
        [1, 4],
    ]

    assert grid.reshape(-1).tolist() == [3, 6, 2, 5, 1, 4]


def test_add_border():
    grid = np.zeros((5, 4), dtype=np.int8)

    add_border(grid)

    # further borders are occupied
    assert (grid[-1, :] == 100).all()
    assert (grid[:, 0] == 100).all()
    assert (grid[:, -1] == 100).all()

    # bottom row and interior is untouched
    assert (grid[0, 1:-1] == 0).all()
    assert (grid[1:-1, 1:-1] == 0).all()


def test_inflate_grid():
    params = InflationParams(
        inflation_radius_cells=1,
        inflation_falloff_radius_cells=2,
        inflation_decay_factor=0.5,
    )

    grid = np.zeros((7, 7), dtype=np.int8)
    grid[3, 3] = 100

    out = inflate_grid(grid, params)

    assert out[3, 3] == 100

    # hard radius (distance = 1)
    assert out[3, 4] == 100
    assert out[3, 2] == 100
    assert out[4, 3] == 100
    assert out[2, 3] == 100

    # soft radius (distance = 2)
    assert out[3, 5] == 50
    assert out[3, 1] == 50
    assert out[5, 3] == 50
    assert out[1, 3] == 50

    # outside r_soft
    assert out[3, 6] == 0
    assert out[0, 3] == 0

