from dataclasses import dataclass, field
import dataclasses
import itertools
import math
import heapq
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.node import Node
from std_msgs.msg import Header

# Constants
ROBOT_FORWARDS_BACKWARDS_POSITION_RELATIVE_TO_BOTTOM_OF_CAMERA_VIEW = -0.60
DRIVABLE_CELL_VALUE = 0

@dataclass(unsafe_hash=True)
class OccupancyGridIndex:
    y: int 
    x: int 

def get_yaw_radians_from_quaternion(q: Quaternion):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def world_point_to_grid_index(
    point: Point, 
    robot_pose: Pose, 
    resolution: float
) -> tuple[int, int]:
    """
    Inverse of convert_occupancy_grid_index_to_meters.
    Converts a world point (meters) into the robot-relative grid index (y, x).
    """
    dx = point.x - robot_pose.position.x
    dy = point.y - robot_pose.position.y
    
    yaw = get_yaw_radians_from_quaternion(robot_pose.orientation)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    # Inverse rotation
    # x_rot = dx * cos + dy * sin
    # y_rot = -dx * sin + dy * cos
    # (derivation based on standard rotation matrix transpose)
    
    # Note: The original conversion was:
    # rot_x = grid_x * cos - grid_y * sin
    # rot_y = grid_x * sin + grid_y * cos
    
    # Solving for grid_x, grid_y:
    grid_x_meters = dx * cos_yaw + dy * sin_yaw
    grid_y_meters = -dx * sin_yaw + dy * cos_yaw
    
    return (
        int(round(grid_y_meters / resolution)), # y index
        int(round(grid_x_meters / resolution))  # x index
    )

def convert_occupancy_grid_index_to_meters(
    occupancy_grid_resolution: float,
    index: OccupancyGridIndex,
    robot_pose: Pose
) -> Point:
    """Used only for final path generation, not in the hot loop."""
    yaw_radians = get_yaw_radians_from_quaternion(robot_pose.orientation)
    
    # Standard rotation
    rx = index.x * math.cos(yaw_radians) - index.y * math.sin(yaw_radians)
    ry = index.x * math.sin(yaw_radians) + index.y * math.cos(yaw_radians)

    return Point(
        x=rx * occupancy_grid_resolution + robot_pose.position.x,
        y=ry * occupancy_grid_resolution + robot_pose.position.y
    )

def find_closest_drivable_point(occupancy_grid: OccupancyGrid) -> tuple[int, int] | None:
    """Finds closest drivable point using raw data access for speed."""
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    resolution = occupancy_grid.info.resolution
    data = occupancy_grid.data
    
    # Pre-calculate offsets to map (y, x) to data index
    # data_index = (x + x_offset) * width + (-y + y_offset)
    x_offset = int(ROBOT_FORWARDS_BACKWARDS_POSITION_RELATIVE_TO_BOTTOM_OF_CAMERA_VIEW / resolution)
    y_offset = width // 2

    # Start at (0, offset) which corresponds to robot center
    start_x = -x_offset
    start_y = 0
    
    queue = deque([(start_y, start_x)])
    visited = {(start_y, start_x)}
    
    while queue:
        cy, cx = queue.popleft()
        
        # Check bounds and drivability inline
        # Transform to raw grid coordinates
        raw_x = cx + x_offset
        raw_y = -cy + y_offset
        
        if 0 <= raw_x < height and 0 <= raw_y < width:
            idx = raw_x * width + raw_y
            if data[idx] == DRIVABLE_CELL_VALUE:
                return (cy, cx)
        
        # Expand neighbors
        for dy, dx in [(0, 1), (1, 1), (-1, 1), (1, 0), (-1, 0)]:
            ny, nx = cy + dy, cx + dx
            
            # Quick bounds check on the LOGICAL coordinates to prevent infinite spiraling
            # (Optional but good practice, keeping it loose for now)
            if (ny, nx) not in visited:
                # Only add if it maps to within the grid roughly
                # (Strict checking happens at start of next loop iteration)
                test_raw_x = nx + x_offset
                test_raw_y = -ny + y_offset
                if 0 <= test_raw_x < height and 0 <= test_raw_y < width:
                    visited.add((ny, nx))
                    queue.append((ny, nx))
    return None

def has_line_of_sight_fast(
    data, width, height, x_offset, y_offset, 
    y0, x0, y1, x1
) -> bool:
    """
    Optimized Bresenham's algorithm. 
    Operates on logical (y, x) but accesses 'data' array directly.
    """
    dy = abs(y1 - y0)
    dx = abs(x1 - x0)
    x = x0
    y = y0
    
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    
    # Determine which raw index to check
    # raw_x = x + x_offset
    # raw_y = -y + y_offset
    # index = raw_x * width + raw_y
    
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            raw_x = x + x_offset
            raw_y = -y + y_offset
            
            # Inline Bounds Check
            if not (0 <= raw_x < height and 0 <= raw_y < width):
                return False
            
            # Inline Drivability Check
            if data[raw_x * width + raw_y] != DRIVABLE_CELL_VALUE:
                return False
                
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            raw_x = x + x_offset
            raw_y = -y + y_offset
            
            if not (0 <= raw_x < height and 0 <= raw_y < width):
                return False

            if data[raw_x * width + raw_y] != DRIVABLE_CELL_VALUE:
                return False
                
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy

    # Check final point
    raw_x = x + x_offset
    raw_y = -y + y_offset
    if not (0 <= raw_x < height and 0 <= raw_y < width):
        return False
    if data[raw_x * width + raw_y] != DRIVABLE_CELL_VALUE:
        return False
        
    return True

from collections import deque

def generate_path_occupancy_grid_indices(
    goal_selection_node: Node,
    occupancy_grid: OccupancyGrid,
    start_tuple: tuple[int, int],
    robot_pose: Pose,
    waypoint_meters: Point,
):
    """
    Optimized Theta* Path Planner.
    Uses tuples (y, x) internally and avoids repeated coordinate transforms.
    """
    # 1. Setup Grid Parameters
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    resolution = occupancy_grid.info.resolution
    data = occupancy_grid.data
    
    x_offset = int(ROBOT_FORWARDS_BACKWARDS_POSITION_RELATIVE_TO_BOTTOM_OF_CAMERA_VIEW / resolution)
    y_offset = width // 2

    # 2. Convert Goal to Grid Coordinates ONCE
    # This allows us to use simple Euclidean distance on indices as the heuristic
    goal_tuple = world_point_to_grid_index(waypoint_meters, robot_pose, resolution)
    goal_y, goal_x = goal_tuple

    # Heuristic function (Euclidean distance in grid cells)
    # We multiply by resolution later, so purely index distance is fine for priority
    def heuristic(y, x):
        return math.hypot(y - goal_y, x - goal_x)

    # 3. Initialize Search
    priority_queue = []
    
    # cost_so_far: (y, x) -> float
    cost_so_far = {}
    came_from = {} # (y, x) -> (parent_y, parent_x)

    start_h = heuristic(*start_tuple)
    heapq.heappush(priority_queue, (start_h, start_tuple))
    
    cost_so_far[start_tuple] = 0.0
    came_from[start_tuple] = start_tuple

    best_goal_tuple = start_tuple
    best_goal_dist = start_h

    # 4. Main Loop
    while priority_queue:
        _, current = heapq.heappop(priority_queue)
        cy, cx = current

        # Early Exit: If we reached the exact goal grid cell (unlikely but possible)
        if current == goal_tuple:
            best_goal_tuple = current
            break

        # Track closest drivable point to goal seen so far
        # (Useful if the actual goal is unreachable/blocked)
        dist_to_goal = heuristic(cy, cx)
        if dist_to_goal < best_goal_dist:
            best_goal_dist = dist_to_goal
            best_goal_tuple = current

        # Explore Neighbors
        # Standard 8-connected grid
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            ny, nx = cy + dy, cx + dx
            neighbor = (ny, nx)

            # --- Inline Bounds & Collision Check ---
            raw_nx = nx + x_offset
            raw_ny = -ny + y_offset
            
            # Check bounds
            if not (0 <= raw_nx < height and 0 <= raw_ny < width):
                continue
            
            # Check collision
            if data[raw_nx * width + raw_ny] != DRIVABLE_CELL_VALUE:
                continue

            # --- Theta* Logic ---
            parent = came_from[current]
            py, px = parent
            
            # Check LOS from Parent -> Neighbor (Shortcut)
            # Only attempt shortcut if parent is different from current
            shortcut_worked = False
            
            if parent != current:
                # Use optimized fast LOS check
                if has_line_of_sight_fast(data, width, height, x_offset, y_offset, py, px, ny, nx):
                    # Path 2: Parent -> Neighbor
                    dist_parent_neighbor = math.hypot(ny - py, nx - px) * resolution
                    new_cost = cost_so_far[parent] + dist_parent_neighbor
                    
                    if new_cost < cost_so_far.get(neighbor, float('inf')):
                        cost_so_far[neighbor] = new_cost
                        came_from[neighbor] = parent
                        priority = new_cost + (heuristic(ny, nx) * resolution)
                        heapq.heappush(priority_queue, (priority, neighbor))
                    shortcut_worked = True

            # If shortcut didn't work or wasn't tried, use Standard A* (Current -> Neighbor)
            if not shortcut_worked:
                dist_current_neighbor = math.hypot(dy, dx) * resolution
                new_cost = cost_so_far[current] + dist_current_neighbor
                
                if new_cost < cost_so_far.get(neighbor, float('inf')):
                    cost_so_far[neighbor] = new_cost
                    came_from[neighbor] = current
                    priority = new_cost + (heuristic(ny, nx) * resolution)
                    heapq.heappush(priority_queue, (priority, neighbor))

    goal_selection_node.get_logger().info(f"Path found to {best_goal_tuple} (dist {best_goal_dist:.2f})")

    # 5. Reconstruct Path
    backtrace = []
    curr = best_goal_tuple
    while came_from[curr] != curr:
        backtrace.append(OccupancyGridIndex(y=curr[0], x=curr[1]))
        curr = came_from[curr]
    backtrace.append(OccupancyGridIndex(y=start_tuple[0], x=start_tuple[1]))

    return reversed(backtrace)

def generate_path(
    goal_selection_node: Node,
    occupancy_grid: OccupancyGrid,
    robot_pose: Pose,
    waypoint_meters: Point,
) -> Path:
    
    # 1. Find Start (Tuple)
    start_tuple = find_closest_drivable_point(occupancy_grid)
    assert start_tuple is not None, "Could not find drivable area in front of robot!"

    # 2. Run Optimized Theta*
    path_indices = generate_path_occupancy_grid_indices(
        goal_selection_node=goal_selection_node,
        occupancy_grid=occupancy_grid,
        start_tuple=start_tuple,
        robot_pose=robot_pose,
        waypoint_meters=waypoint_meters
    )

    # 3. Convert to Meters
    poses = [
        PoseStamped(
            header=Header(frame_id="odom"),
            pose=Pose(
                position=convert_occupancy_grid_index_to_meters(
                    occupancy_grid.info.resolution,
                    index,
                    robot_pose
                )
            )
        )
        for index in path_indices
    ]

    return Path(
        header=Header(frame_id="odom"),
        poses=poses
    )