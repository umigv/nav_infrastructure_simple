import math
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_utils.world_occupancy_grid import WorldOccupancyGrid, CellState
from nav_utils.geometry import make_quarternion_from_yaw, make_pose

def make_occupancy_grid() -> OccupancyGrid:
    return OccupancyGrid(
        info=MapMetaData(
            resolution=0.5,
            width=6,
            height=6,
            origin=make_pose(x=2.269615242270663, y=1.0009618943233418, yaw=math.pi/6)
        ),
        data=np.zeros(36).astype(np.int8).tolist()
    )

def point_is_close(pointA: Point, pointB: Point) -> bool:
    """Checks if two points are close"""
    return math.isclose(pointA.x, pointB.x, abs_tol=0.01) and math.isclose(pointA.y, pointB.y, abs_tol=0.01)

def test_world_to_grid_index():
    grid = WorldOccupancyGrid(make_occupancy_grid())

    assert grid._world_to_grid_index(Point(x=3.25, y=3.5, z=0.0)) == (4, 3)
    assert grid._world_to_grid_index(Point(x=2.0, y=1.0, z=0.0)) == (-1, 0)

def test_grid_index_center_to_world():
    grid = WorldOccupancyGrid(make_occupancy_grid())

    assert point_is_close(grid._grid_index_center_to_world(2, 0), Point(x=3.2271, y=1.8425, z=0.0))
    assert point_is_close(grid._grid_index_center_to_world(2, -1), Point(x=3.4771, y=1.4095, z=0.0))

def test_state():
    occupancy_grid = make_occupancy_grid()
    occupancy_grid.data[22] = CellState.OCCUPIED
    
    grid = WorldOccupancyGrid(occupancy_grid)

    assert grid.state(Point(x=2.5, y=2.0, z=0.0)) == CellState.FREE
    assert grid.state(Point(x=3.25, y=3.5, z=0.0)) == CellState.OCCUPIED
    assert grid.state(Point(x=2.0, y=1.0, z=0.0)) == CellState.UNKNOWN

def test_neighbors():
    grid = WorldOccupancyGrid(make_occupancy_grid())

    point = Point(x=3.25, y=3.5, z=0.0)
    assert grid._world_to_grid_index(point) == (4, 3)

    neighbors4 = list(grid.neighbors4(point))
    assert len(neighbors4) == 4
    assert {grid._world_to_grid_index(q) for q in neighbors4} == {(5, 3), (4, 4), (4, 2), (3, 3)}

    neighbors8 = list(grid.neighbors8(point))
    assert len(neighbors8) == 8
    assert {grid._world_to_grid_index(q) for q in neighbors8} == {
        (3, 2), (4, 2), (5, 2),
        (3, 3),         (5, 3),
        (3, 4), (4, 4), (5, 4),
    }

    neighbors_forward = list(grid.neighbors_forward(point))
    assert len(neighbors_forward) == 5
    assert [grid._world_to_grid_index(q) for q in neighbors_forward] == [
        (5, 3),  # forward
        (5, 4),  # forward-left
        (5, 2),  # forward-right
        (4, 4),  # left
        (4, 2),  # right
    ]

def test_hash_key_same_grid():
    grid = WorldOccupancyGrid(make_occupancy_grid())

    point1 = Point(x=3.25, y=3.5, z=0.0)
    point2 = Point(x=3.25, y=3.5, z=0.0)
    assert grid.hash_key(point1) == grid.hash_key(point2)

def test_hash_key_unique_indices():
    grid = WorldOccupancyGrid(make_occupancy_grid())

    # Arbitrary selection of indices to check for hash collision
    indices = [(4, 3), (5, 3), (4, 4), (4, 2), (3, 3), (-1, 0), (2, -1), (-2, -3)]

    keys = []
    for ix, iy in indices:
        world = grid._grid_index_center_to_world(ix, iy)
        assert grid._world_to_grid_index(world) == (ix, iy)
        keys.append(grid.hash_key(world))

    assert len(set(keys)) == len(keys)
