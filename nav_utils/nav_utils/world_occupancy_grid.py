from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Pose
import numpy as np
import math
from typing import Iterator
from enum import IntEnum
from nav_utils.geometry import get_yaw_radians_from_quaternion, rotate_by_yaw

class CellState(IntEnum):
    UNKNOWN    = -1
    FREE       = 0
    OCCUPIED   = 100

    @property
    def drivable(self) -> bool:
        return self == CellState.FREE

class WorldOccupancyGrid:
    # Robot pose in the world frame (e.g. odom or map) at the time the grid was captured.
    _pose: Pose
    # Cached yaw (rotation about +Z, radians) extracted from `_pose.orientation`.
    _yaw: float
    # 2D occupancy matrix indexed as _grid[grid_x, grid_y].
    # Values are raw occupancy values (-1 unknown, 0 free, 100 occupied).
    # The matrix is rotated and reshaped such that +X is forward and +Y is left
    # in the robot frame.
    _grid: np.ndarray
    # Number of grid cells in the +X (forward) direction.
    _width: int
    # Number of grid cells in the +Y (left) direction.
    _height: int
    # Size of one grid cell in meters (meters per cell).
    _resolution: float
    # Forward offset (meters) from the robot origin to the grid's x=0 column.
    _robot_forward_offset_m: float
    # Half of the grid height in meters.
    _half_height: float

    def __init__(self, grid: OccupancyGrid, pose: Pose, robot_forward_offset_m: float) -> None:
        self._pose = pose
        self._yaw = get_yaw_radians_from_quaternion(self._pose.orientation)

        matrix = np.asarray(grid.data, dtype=np.int8).reshape((grid.info.width, grid.info.height), order='F')
        matrix = np.rot90(matrix, k=-1)
        self._grid = matrix
        self._width, self._height = matrix.shape
        self._resolution = grid.info.resolution

        self._robot_forward_offset_m = robot_forward_offset_m
        self._half_height = self._height * self._resolution / 2.0

    def state(self, point: Point) -> CellState:
        grid_x, grid_y = self.world_to_grid_index(point)

        if not(0 <= grid_x < self._width and 0 <= grid_y < self._height):
            return CellState.UNKNOWN

        return CellState(int(self._grid[grid_x, grid_y]))

    def neighbors4(self, point: Point) -> Iterator[Point]:
        x, y = self.world_to_grid_index(point)

        for dx, dy in ((1, 0), (0, 1), (0, -1), (-1, 0)):
                yield self.grid_index_center_to_world(x + dx, y + dy)

    def neighbors8(self, point: Point) -> Iterator[Point]:
        x, y = self.world_to_grid_index(point)

        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                yield self.grid_index_center_to_world(x + dx, y + dy)

    def neighbors_forward(self, point: Point) -> Iterator[Point]:
        x, y = self.world_to_grid_index(point)

        candidates = [
            (1, 0),  # forward
            (1, 1),  # forward-left
            (1, -1), # forward-right
            (0, 1),  # left
            (0, -1), # right
        ]

        for dx, dy in candidates:
            yield self.grid_index_center_to_world(x + dx, y + dy)

    def hash_key(self, point: Point) -> int:
        x, y = self.world_to_grid_index(point)

        # map signed to unsigned by mapping positive to even and negative to odd
        zx = 2 * x if x >= 0 else -2 * x - 1
        zy = 2 * y if y >= 0 else -2 * y - 1

        # cantor pairing (https://en.wikipedia.org/wiki/Pairing_function)
        return (zx + zy) * (zx + zy + 1) // 2 + zy

    def world_to_grid_index(self, world: Point) -> tuple[int, int]:
        robot = rotate_by_yaw(Point(x = world.x - self._pose.position.x,
                            y = world.y - self._pose.position.y, 
                            z = 0.0), -self._yaw)
        
        grid = Point(x = robot.x - self._robot_forward_offset_m, 
                     y = robot.y + self._half_height, 
                     z = 0.0)
        
        return math.floor(grid.x / self._resolution), math.floor(grid.y / self._resolution)

    def grid_index_center_to_world(self, grid_x: int, grid_y: int) -> Point:        
        grid = Point(x = (grid_x + 0.5) * self._resolution, 
                     y = (grid_y + 0.5) * self._resolution, 
                     z = 0.0) 
        
        robot = Point(x = grid.x + self._robot_forward_offset_m,
                      y = grid.y - self._half_height,
                      z = 0.0)

        robot_rotated = rotate_by_yaw(robot, self._yaw)

        return Point(x = robot_rotated.x + self._pose.position.x, 
                    y = robot_rotated.y + self._pose.position.y, 
                    z = 0.0)
