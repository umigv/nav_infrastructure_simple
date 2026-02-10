import math

import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav_utils.geometry import get_yaw_radians_from_quaternion, make_pose
from occupancy_grid_transform.occupancy_grid_transform_impl import (
    compute_origin_pose,
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


def test_compute_origin_pose():
    origin = compute_origin_pose(
        odom=make_pose(x=1.0, y=2.0, yaw=math.pi / 6),
        robot_forward_offset_m=0.60,
        grid_height_cells=6,
        resolution=0.5,
    )

    assert math.isclose(origin.position.x, 2.269615242270663)
    assert math.isclose(origin.position.y, 1.0009618943233418)
    assert math.isclose(get_yaw_radians_from_quaternion(origin.orientation), math.pi / 6)
