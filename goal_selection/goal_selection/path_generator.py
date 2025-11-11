from dataclasses import dataclass
import dataclasses
import itertools
import json
import math
import sys
from collections import deque
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Header

@dataclass(unsafe_hash=True)
class OccupancyGridIndex:
    x: int
    y: int

NULL_OCC_GRID_INDEX = OccupancyGridIndex(x=-1, y=-1)
ROBOT_POSITION_IN_OCC_GRID = OccupancyGridIndex(x=12, y=77)
DRIVABLE_CELL_VALUE = 0

def index_occupancy_grid(occupancy_grid: OccupancyGrid, index: OccupancyGridIndex):
    return occupancy_grid.data[index.x * occupancy_grid.info.height + index.y]

def convert_occupancy_grid_coordinates_to_robot_relative_position(
    occupancy_grid_resolution: float,
    occupancy_grid_coordinates: OccupancyGridIndex,
    robot_position: Point
) -> Point:
    return Point(
        x=(occupancy_grid_coordinates.x - ROBOT_POSITION_IN_OCC_GRID.x) * occupancy_grid_resolution + robot_position.x,
        y=(occupancy_grid_coordinates.y - ROBOT_POSITION_IN_OCC_GRID.y) * occupancy_grid_resolution + robot_position.y
    )

def inflate_occupancy_grid(occupancy_grid: OccupancyGrid) -> OccupancyGrid:
    print("Warning: inflate_occupancy_grid called but not implemented, returning uninflated occupancy grid!", file=sys.stderr)
    return occupancy_grid

# The robot is in unknown space in the occupancy grid, so we traverse forwards until we find the first drivable node
def find_closest_drivable_node(occupancy_grid: OccupancyGrid) -> OccupancyGridIndex | None:
    current_position = dataclasses.replace(ROBOT_POSITION_IN_OCC_GRID)
    while current_position.x < occupancy_grid.info.height:
        if index_occupancy_grid(occupancy_grid, current_position) == DRIVABLE_CELL_VALUE:
            return current_position

        current_position.x += 1
    
    return None

def node_cost(
    occupancy_grid_resolution: float,
    node: Point,
    robot_position: Point,
    waypoint_robot_relative: Point
) -> float:
    node_robot_relative_coords = convert_occupancy_grid_coordinates_to_robot_relative_position(
        occupancy_grid_resolution,
        node,
        robot_position
    )

    return math.sqrt(
        (node_robot_relative_coords.x - waypoint_robot_relative.x) ** 2
        + (node_robot_relative_coords.y - waypoint_robot_relative.y) ** 2
    )

@dataclass
class GoalAndCost:
    goal: OccupancyGridIndex
    cost: float

def generate_path(
    occupancy_grid: OccupancyGrid,
    robot_pose: Pose,
    waypoint_robot_relative: Point,
) -> Path:
    inflated_occupancy_grid = inflate_occupancy_grid(occupancy_grid)

    visited: dict[OccupancyGridIndex, OccupancyGridIndex] = {}
    search_container: deque[OccupancyGridIndex] = deque()

    start_node = find_closest_drivable_node(inflated_occupancy_grid)

    assert start_node is not None, "Could not find drivable area in front of robot!"

    search_container.append(start_node)
    visited[start_node] = NULL_OCC_GRID_INDEX

    goal_and_cost: GoalAndCost | None = None

    while len(search_container) > 0:
        node = search_container.popleft()

        cost = node_cost(
            occupancy_grid_resolution=inflated_occupancy_grid.info.resolution,
            node=node,
            robot_position=robot_pose.position,
            waypoint_robot_relative=waypoint_robot_relative
        )

        if goal_and_cost is None or cost < goal_and_cost.cost:
            goal_and_cost = GoalAndCost(
                goal=node,
                cost=cost
            )

        for dx, dy in itertools.product([-1, 0, 1], repeat=2):
            potential_position = OccupancyGridIndex(
                x=node.x + dx,
                y=node.y + dy
            )

            if potential_position.x < 0 or potential_position.x >= inflated_occupancy_grid.info.height\
                or potential_position.y < 0 or potential_position.y >= inflated_occupancy_grid.info.width:
                continue

            if index_occupancy_grid(inflated_occupancy_grid, potential_position) != DRIVABLE_CELL_VALUE \
                or potential_position in visited:
                continue
            
            search_container.append(potential_position)
            visited[potential_position] = node

    backtrace = []
    current_backtrace_node = dataclasses.replace(goal_and_cost.goal)
    while visited[current_backtrace_node] != NULL_OCC_GRID_INDEX:
        backtrace.append(current_backtrace_node)
        current_backtrace_node = visited[current_backtrace_node]

    return Path(
        header=Header(
            frame_id="map"
        ),
        poses=[
            PoseStamped(
                header=Header(
                    frame_id="map"
                ),
                pose=Pose(
                    position=convert_occupancy_grid_coordinates_to_robot_relative_position(
                        inflated_occupancy_grid.info.resolution,
                        backtrace_node,
                        robot_pose.position
                    )
                )
            )
            for backtrace_node in reversed(backtrace)
        ]
    )