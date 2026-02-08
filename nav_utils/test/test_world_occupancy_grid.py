import math
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_utils.world_occupancy_grid import WorldOccupancyGrid, CellState

def quat_from_yaw(yaw: float) -> Quaternion:
    """ROS quaternion for yaw about +Z."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def make_pose(x: float, y: float, yaw: float) -> Pose:
    """Make 2d robot pose"""
    p = Pose()
    p.position = Point(x=x, y=y, z=0.0)
    p.orientation = quat_from_yaw(yaw)
    return p

def make_grid_from_target_matrix(target_matrix: np.ndarray, resolution: float = 1.0) -> OccupancyGrid:
    """Builds an occupancy grid in cv's frame from a matrix in our frame"""
    A = np.rot90(target_matrix, k=1)

    grid = OccupancyGrid()
    grid.info.resolution = float(resolution)
    grid.info.height, grid.info.width = target_matrix.shape
    grid.data = A.flatten(order="F").astype(np.int8).tolist()
    return grid

def point_is_close(pointA: Point, pointB: Point) -> bool:
    """Checks if two points are close"""
    return math.isclose(pointA.x, pointB.x, abs_tol=0.01) and math.isclose(pointA.y, pointB.y, abs_tol=0.01)

def test_world_to_grid_index():
    grid = WorldOccupancyGrid(
        grid=make_grid_from_target_matrix(np.zeros((6, 6), dtype=np.int8), resolution=0.5), 
        pose=make_pose(x=1.0, y=2.0, yaw=math.pi/6), 
        robot_forward_offset_m=0.6
    )

    assert grid._world_to_grid_index(Point(x=3.25, y=3.5, z=0.0)) == (4, 3)
    assert grid._world_to_grid_index(Point(x=2.0, y=1.0, z=0.0)) == (-1, 0)

def test_grid_index_center_to_world():
    grid = WorldOccupancyGrid(
        grid=make_grid_from_target_matrix(np.zeros((6, 6), dtype=np.int8), resolution=0.5), 
        pose=make_pose(x=1.0, y=2.0, yaw=math.pi/6), 
        robot_forward_offset_m=0.6
    )

    assert point_is_close(grid._grid_index_center_to_world(2, 0), Point(x=3.2271, y=1.8425, z=0.0))
    assert point_is_close(grid._grid_index_center_to_world(2, -1), Point(x=3.4771, y=1.4095, z=0.0))

def test_state():
    matrix = np.zeros((6, 6), dtype=np.int8)
    matrix[4, 3] = 100
    
    grid = WorldOccupancyGrid(
        grid=make_grid_from_target_matrix(matrix, resolution=0.5), 
        pose=make_pose(x=1.0, y=2.0, yaw=math.pi/6), 
        robot_forward_offset_m=0.6
    )

    assert grid.state(Point(x=2.5, y=2.0, z=0.0)).drivable
    assert not grid.state(Point(x=3.25, y=3.5, z=0.0)).drivable
    assert grid.state(Point(x=2.0, y=1.0, z=0.0)).unknown

def test_neighbors():
    grid = WorldOccupancyGrid(
        grid=make_grid_from_target_matrix(np.zeros((6, 6), dtype=np.int8), resolution=0.5),
        pose=make_pose(x=1.0, y=2.0, yaw=math.pi/6),
        robot_forward_offset_m=0.6
    )

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
    grid = WorldOccupancyGrid(
        grid=make_grid_from_target_matrix(np.zeros((6, 6), dtype=np.int8), resolution=0.5),
        pose=make_pose(x=1.0, y=2.0, yaw=math.pi/6),
        robot_forward_offset_m=0.6
    )

    point1 = Point(x=3.25, y=3.5, z=0.0)
    point2 = Point(x=3.25, y=3.5, z=0.0)
    assert grid.hash_key(point1) == grid.hash_key(point2)

def test_hash_key_unique_indices():
    grid = WorldOccupancyGrid(
        grid=make_grid_from_target_matrix(np.zeros((6, 6), dtype=np.int8), resolution=0.5),
        pose=make_pose(x=1.0, y=2.0, yaw=math.pi/6),
        robot_forward_offset_m=0.6
    )

    # Arbitrary selection of indices to check for hash collision
    indices = [(4, 3), (5, 3), (4, 4), (4, 2), (3, 3), (-1, 0), (2, -1), (-2, -3)]

    keys = []
    for ix, iy in indices:
        world = grid._grid_index_center_to_world(ix, iy)
        assert grid._world_to_grid_index(world) == (ix, iy)
        keys.append(grid.hash_key(world))

    assert len(set(keys)) == len(keys)
