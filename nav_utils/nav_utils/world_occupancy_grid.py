from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
import math
from typing import Iterator
from dataclasses import dataclass
from nav_utils.geometry import get_yaw_radians_from_quaternion, rotate_by_yaw
from typing import ClassVar

@dataclass(frozen=True)
class CellState:
    """
    Discrete occupancy state of a grid cell.
    """
    value: int  # -1 (unknown) or 0–100 (probability occupied)

    UNKNOWN_VALUE: ClassVar[int] = -1
    DRIVABLE_THRESHOLD: ClassVar[int] = 30
    SELF_DRIVE_GOAL_VALUE: ClassVar[int] = 127

    def __post_init__(self) -> None:
        if not (0 <= self.value <= 100 or self.value in [CellState.UNKNOWN_VALUE, CellState.SELF_DRIVE_GOAL_VALUE]):
            raise ValueError("CellState value must be within [0, 100] or equal to CellState.UNKNOWN_VALUE or "
                             "CellState.SELF_DRIVE_GOAL_VALUE")

    @classmethod
    def unknown_cell(cls) -> "CellState":
        return cls(cls.UNKNOWN_VALUE)

    @property
    def isDrivable(self) -> bool:
        """
        True if the cell can be traversed by the robot
        """
        return 0 <= self.value <= CellState.DRIVABLE_THRESHOLD or self.isSelfDriveGoal

    @property
    def isUnknown(self) -> bool:
        """
        True if the cell value is unknown
        """
        return self.value == CellState.UNKNOWN_VALUE

    @property
    def isSelfDriveGoal(self) -> bool:
        """
        True if the cell is cell drive goal
        """
        return self.value == CellState.SELF_DRIVE_GOAL_VALUE        

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
            return CellState.unknown_cell()

        return CellState(self._occupancy_grid.data[grid_y * self._occupancy_grid.info.width + grid_x])

    def inBoundPoints(self) -> Iterator[Point]:
        """
        Generate a point in all in bound grids

        Yields:
            World-coordinate points at the centers of neighboring grid cells.
        """
        for x in range(self._occupancy_grid.info.width):
            for y in range(self._occupancy_grid.info.height):
                yield self._grid_index_center_to_world(x, y)

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
