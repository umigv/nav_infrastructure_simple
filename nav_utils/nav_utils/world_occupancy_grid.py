from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
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
    - OCCUPIED (1): Cell is occupied by an obstacle.

    This enum provides a semantic layer over raw occupancy values and is used throughout planning code to reason about 
    traversability.
    """
    
    UNKNOWN    = -1
    FREE       = 0
    OCCUPIED   = 1

    @staticmethod
    def from_occupancy(value: int) -> "CellState":
        """
        Classify a raw ROS occupancy value into a CellState.

        Returns:
            UNKNOWN if value is -1, FREE if value is 0, OCCUPIED if value is between 1 and 100
        """
        if value == -1:
            return CellState.UNKNOWN
        
        if value == 0:
            return CellState.FREE
        
        if 1 <= value <= 100:
            return CellState.OCCUPIED
        
        raise ValueError(f"Invalid occupancy value: {value} (expected -1, 0, or 1..100)")

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
    Continuous world-facing view of an occupancy grid.

    This class wraps a discrete `nav_msgs/msg/OccupancyGrid` and exposes it as a continuous world-coordinate 
    representation. This abstraction allows discrete grid-based search (e.g. A*, BFS) to be expressed entirely in world 
    coordinates, while preserving correct grid semantics.

    Conceptually, the occupancy grid is treated as an infinite world:
    - World points are projected into grid cells on demand.
    - Points outside the underlying grid bounds are treated as UNKNOWN.

    Attributes:
        _occupancy_grid: The occupancy grid.
        _yaw: Cached yaw (radians about +Z) extracted from `_pose.orientation`.
    """

    _occupancy_grid: OccupancyGrid
    _yaw: float

    def __init__(self, grid: OccupancyGrid) -> None:
        """
        Construct a world-facing view of a discrete occupancy grid.

        The occupancy grid is assumed to be in ROS2 conventions, where
        - +x is forward
        - +y is left
        - data is stored in row major order

        Args:
            grid: Occupancy grid message. 
        """
        self._occupancy_grid = grid
        self._yaw = get_yaw_radians_from_quaternion(self._occupancy_grid.info.origin.orientation)

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

        if not(0 <= grid_x < self._occupancy_grid.info.width and 0 <= grid_y < self._occupancy_grid.info.height):
            return CellState.UNKNOWN

        return CellState.from_occupancy(self._occupancy_grid.data[grid_y * self._occupancy_grid.info.width + grid_x])

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

        Args:
            world: World-coordinate point.

        Returns:
            (grid_x, grid_y) integer indices corresponding to the grid cell containing the point.
        """
        grid = rotate_by_yaw(Point(x = world.x - self._occupancy_grid.info.origin.position.x,
                                   y = world.y - self._occupancy_grid.info.origin.position.y, 
                                   z = 0.0), -self._yaw)
        
        return math.floor(grid.x / self._occupancy_grid.info.resolution), \
               math.floor(grid.y / self._occupancy_grid.info.resolution)

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
        grid = Point(x = (grid_x + 0.5) * self._occupancy_grid.info.resolution, 
                     y = (grid_y + 0.5) * self._occupancy_grid.info.resolution, 
                     z = 0.0) 

        rotated = rotate_by_yaw(grid, self._yaw)

        return Point(x = rotated.x + self._occupancy_grid.info.origin.position.x, 
                     y = rotated.y + self._occupancy_grid.info.origin.position.y, 
                     z = 0.0)
