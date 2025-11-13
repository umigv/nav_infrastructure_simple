from dataclasses import dataclass
import dataclasses
import itertools
import json
import math
import sys
from collections import deque
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header

@dataclass(unsafe_hash=True)
class OccupancyGridIndex:
    x: int # +x = right
    y: int # +y = forward

NULL_OCC_GRID_INDEX = OccupancyGridIndex(x=-1, y=-1)
ROBOT_POSITION_IN_OCC_GRID = OccupancyGridIndex(x=77, y=12)
DRIVABLE_CELL_VALUE = 0

def index_occupancy_grid(occupancy_grid: OccupancyGrid, index: OccupancyGridIndex):
    return occupancy_grid.data[index.y * occupancy_grid.info.width + index.x]

def get_yaw_radians_from_quaternion(q: Quaternion):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def convert_occupancy_grid_coordinates_to_robot_relative_position(
    occupancy_grid_resolution: float,
    occupancy_grid_coordinates: OccupancyGridIndex,
    robot_pose: Pose
) -> Point:
    yaw_radians = get_yaw_radians_from_quaternion(robot_pose.orientation)

    occ_point_with_robot_occ_origin = OccupancyGridIndex(
        x=occupancy_grid_coordinates.x - ROBOT_POSITION_IN_OCC_GRID.x,
        y=occupancy_grid_coordinates.y - ROBOT_POSITION_IN_OCC_GRID.y
    )

    rotated_occ_point_with_robot_occ_origin = Point(
        x=occ_point_with_robot_occ_origin.x * math.cos(yaw_radians) - occ_point_with_robot_occ_origin.y * math.sin(yaw_radians),
        y=occ_point_with_robot_occ_origin.x * math.sin(yaw_radians) + occ_point_with_robot_occ_origin.y * math.cos(yaw_radians)
    )

    return Point(
        x=rotated_occ_point_with_robot_occ_origin.x * occupancy_grid_resolution + robot_pose.position.x,
        y=rotated_occ_point_with_robot_occ_origin.y * occupancy_grid_resolution + robot_pose.position.y
    )

# The robot is in unknown space in the occupancy grid, 
# so we traverse forwards until we find the first drivable node.
#
# If going forward is not enough, we then traverse left and right as well
def find_closest_drivable_node(occupancy_grid: OccupancyGrid) -> OccupancyGridIndex | None:
    visited: set[OccupancyGridIndex] = set()
    search_container: deque[OccupancyGridIndex] = deque()

    current_position = dataclasses.replace(ROBOT_POSITION_IN_OCC_GRID)
    visited.add(current_position)
    search_container.append(current_position)
    
    while len(search_container) > 0:
        node = search_container.popleft()

        for dx, dy in [
            (0, 1),
            (1, 1),
            (-1, 1),
            (1, 0),
            (-1, 0)
        ]:
            potential_position = OccupancyGridIndex(
                x=node.x + dx,
                y=node.y + dy
            )

            if potential_position.x < 0 or potential_position.x >= occupancy_grid.info.width\
                or potential_position.y < 0 or potential_position.y >= occupancy_grid.info.height:
                continue

            if potential_position in visited:
                continue

            if index_occupancy_grid(occupancy_grid, potential_position) == DRIVABLE_CELL_VALUE:
                return potential_position
            
            search_container.append(potential_position)
            visited.add(potential_position)
    
    return None

def node_cost(
    occupancy_grid_resolution: float,
    node: OccupancyGridIndex,
    robot_pose: Pose,
    waypoint_robot_relative: Point
) -> float:
    node_robot_relative_coords = convert_occupancy_grid_coordinates_to_robot_relative_position(
        occupancy_grid_resolution,
        node,
        robot_pose
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
    visited: dict[OccupancyGridIndex, OccupancyGridIndex] = {}
    search_container: deque[OccupancyGridIndex] = deque()

    start_node = find_closest_drivable_node(occupancy_grid)

    assert start_node is not None, "Could not find drivable area in front of robot!"

    search_container.append(start_node)
    visited[start_node] = NULL_OCC_GRID_INDEX

    goal_and_cost: GoalAndCost | None = None

    while len(search_container) > 0:
        node = search_container.popleft()

        cost = node_cost(
            occupancy_grid_resolution=occupancy_grid.info.resolution,
            node=node,
            robot_pose=robot_pose,
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

            if potential_position.x < 0 or potential_position.x >= occupancy_grid.info.width\
                or potential_position.y < 0 or potential_position.y >= occupancy_grid.info.height:
                continue

            if index_occupancy_grid(occupancy_grid, potential_position) != DRIVABLE_CELL_VALUE \
                or potential_position in visited:
                continue
            
            search_container.append(potential_position)
            visited[potential_position] = node

    backtrace: list[OccupancyGridIndex] = []
    current_backtrace_node = dataclasses.replace(goal_and_cost.goal)
    while visited[current_backtrace_node] != NULL_OCC_GRID_INDEX:
        backtrace.append(current_backtrace_node)
        current_backtrace_node = visited[current_backtrace_node]

    return Path(
        header=Header(
            frame_id="odom"
        ),
        poses=[
            PoseStamped(
                header=Header(
                    frame_id="odom"
                ),
                pose=Pose(
                    position=convert_occupancy_grid_coordinates_to_robot_relative_position(
                        occupancy_grid.info.resolution,
                        backtrace_node,
                        robot_pose
                    )
                )
            )
            for backtrace_node in reversed(backtrace)
        ]
    )