from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Pose
import numpy as np
import math
from typing import Iterator
from enum import IntEnum
from nav_utils.geometry import get_yaw_radians_from_quaternion, rotate_by_yaw

class CellState(IntEnum):
    """
    Discrete occupancy state of a grid cell.

    Values follow the ROS occupancy grid convention:
    - UNKNOWN  (-1): Cell occupancy is unknown or outside the grid bounds.
    - FREE      (0): Cell is known to be free and traversable.
    - OCCUPIED (100): Cell is occupied by an obstacle.

    This enum provides a semantic layer over raw occupancy values and is used throughout planning code to reason about 
    traversability.
    """
    
    UNKNOWN    = -1
    FREE       = 0
    OCCUPIED   = 100

    @property
    def drivable(self) -> bool:
        """
        Whether this cell can be traversed by the robot.

        Returns: 
            True if the cell state is FREE. UNKNOWN and OCCUPIED grids are treated as non-drivable.
        """
        return self == CellState.FREE

class WorldOccupancyGrid:
    """
    World-facing view of a robot-centric occupancy grid.

    This class wraps a discrete, robot-centric `nav_msgs/msg/OccupancyGrid` and exposes it as a continuous 
    world-coordinate representation. This abstraction allows discrete grid-based search (e.g. A*, BFS) to be expressed 
    entirely in world coordinates, while preserving correct grid semantics.

    Conceptually, the occupancy grid is treated as an infinite world:
    - World points are projected into grid cells on demand.
    - Points outside the underlying grid bounds are treated as UNKNOWN.

    The grid is internally rotated and reshaped to align with the odometry frame:
    - +X points forward from the robot.
    - +Y points to the left of the robot.
    - The grid origin corresponds to the bottom-left corner of the grid.
    - The grid begins `robot_forward_offset_m` meters in front of the robot.

    Attributes:
        _pose: Robot pose in the world frame (e.g. odom/map) at capture time.
        _yaw: Cached yaw (radians about +Z) extracted from `_pose.orientation`.
        _grid: 2D occupancy matrix indexed as `_grid[grid_x, grid_y]` containing raw values
            (-1 unknown, 0 free, 100 occupied), rotated/reshaped to match the conventions above.
        _width: Number of cells in +X (forward) direction.
        _height: Number of cells in +Y (left) direction.
        _resolution: Cell size in meters.
        _robot_forward_offset_m: Forward offset (m) from robot origin to grid x=0 column.
        _half_height: Half of grid height in meters (used to center the grid in +Y).
    """

    _pose: Pose
    _yaw: float
    _grid: np.ndarray
    _width: int
    _height: int
    _resolution: float
    _robot_forward_offset_m: float
    _half_height: float

    def __init__(self, grid: OccupancyGrid, pose: Pose, robot_forward_offset_m: float) -> None:
        """
        Construct a world-facing view of a robot-centric occupancy grid.

        The occupancy grid is assumed to be centered about the robot in +Y and begins robot_forward_offset_m meters in 
        front of the robot in +X. `grid.data` is interpreted as column-major, with index 0 as the top-left cell.

        Args:
            grid: Robot-centric occupancy grid message. 
            pose: Robot pose in the world frame at the time the grid was captured.
            robot_forward_offset_m: Forward offset (meters) from the robot origin to the grid's x=0 column.
        """
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
        """
        Query the occupancy state at a world-coordinate point.

        The point is projected into the underlying occupancy grid. If the projected grid index lies outside the grid 
        bounds, the state is treated as UNKNOWN.

        Args:
            point: World-coordinate point to query.

        Returns:
            CellState corresponding to the occupancy at the given point.
        """
        grid_x, grid_y = self._world_to_grid_index(point)

        if not(0 <= grid_x < self._width and 0 <= grid_y < self._height):
            return CellState.UNKNOWN

        return CellState(int(self._grid[grid_x, grid_y]))

    def neighbors4(self, point: Point) -> Iterator[Point]:
        """
        Generate 4-connected neighboring world points for discrete grid search.

        Neighbors correspond to the centers of the grid cells adjacent to the cell containing `point` in the cardinal 
        directions (±X, ±Y).

        Args:
            point: World-coordinate point whose grid cell is expanded.

        Yields:
            World-coordinate points at the centers of neighboring grid cells.
        """
        x, y = self._world_to_grid_index(point)

        for dx, dy in ((1, 0), (0, 1), (0, -1), (-1, 0)):
            yield self._grid_index_center_to_world(x + dx, y + dy)

    def neighbors8(self, point: Point) -> Iterator[Point]:
        """
        Generate 8-connected neighboring world points for discrete grid search.

        Neighbors correspond to the centers of all adjacent grid cells, including diagonals, around the cell containing 
        `point`.

        Args:
            point: World-coordinate point whose grid cell is expanded.

        Yields:
            World-coordinate points at the centers of neighboring grid cells.
        """
        x, y = self._world_to_grid_index(point)

        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                yield self._grid_index_center_to_world(x + dx, y + dy)

    def neighbors_forward(self, point: Point) -> Iterator[Point]:
        """
        Generate a forward-biased set of neighboring world points.

        This expansion favors motion in front of the robot and is useful for planners that prefer forward progress. 
        The expansion includes: forward, forward-left, forward-right, left, and right neighbors.

        Args:
            point: World-coordinate point whose grid cell is expanded.

        Yields:
            World-coordinate points at the centers of selected neighboring cells.
        """
        x, y = self._world_to_grid_index(point)

        candidates = [
            (1, 0),  # forward
            (1, 1),  # forward-left
            (1, -1), # forward-right
            (0, 1),  # left
            (0, -1), # right
        ]

        for dx, dy in candidates:
            yield self._grid_index_center_to_world(x + dx, y + dy)

    def hash_key(self, point: Point) -> int:
        """
        Compute a stable hash key for the grid cell containing a world point.

        All world points that project into the same grid cell will produce the same hash key. This enables discrete 
        search bookkeeping (e.g. visited sets) without storing raw grid indices or floating-point coordinates.

        Args:
            point: World-coordinate point to hash.

        Returns:
            Integer hash uniquely identifying the corresponding grid cell.
        """
        x, y = self._world_to_grid_index(point)

        # map signed to unsigned by mapping positive to even and negative to odd
        zx = 2 * x if x >= 0 else -2 * x - 1
        zy = 2 * y if y >= 0 else -2 * y - 1

        # cantor pairing (https://en.wikipedia.org/wiki/Pairing_function)
        return (zx + zy) * (zx + zy + 1) // 2 + zy

    def _world_to_grid_index(self, world: Point) -> tuple[int, int]:
        """
        Project a world-coordinate point into discrete grid indices.

        The transformation:
        1. Translates the point into the robot frame
        2. Rotates by -robot yaw to align with the grid
        3. Applies the forward offset and vertical centering
        4. Quantizes into grid indices using the grid resolution

        Args:
            world: World-coordinate point.

        Returns:
            (grid_x, grid_y) integer indices corresponding to the grid cell
            containing the point.
        """
        robot = rotate_by_yaw(Point(x = world.x - self._pose.position.x,
                            y = world.y - self._pose.position.y, 
                            z = 0.0), -self._yaw)
        
        grid = Point(x = robot.x - self._robot_forward_offset_m, 
                     y = robot.y + self._half_height, 
                     z = 0.0)
        
        return math.floor(grid.x / self._resolution), math.floor(grid.y / self._resolution)

    def _grid_index_center_to_world(self, grid_x: int, grid_y: int) -> Point:
        """
        Convert a grid cell index to the world-coordinate position of its center.

        This is the inverse of `_world_to_grid_index` (up to quantization), mapping discrete grid indices back into 
        continuous world space.

        Args:
            grid_x: Grid index in the +X (forward) direction.
            grid_y: Grid index in the +Y (left) direction.

        Returns:
            World-coordinate point at the center of the specified grid cell.
        """
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
