# nav_utils
Small utilities for ROS 2 Python nodes:

## nav_utils.config
Adds a simple config-loading utility for ROS 2.

To use, you will want to create a python dataclass where each field in the config dataclass corresponds to a ROS 2 
parameter key. The field’s default value (if provided) is used as the parameter’s default.

Mapping rules:
- Each dataclass field name corresponds to a ROS 2 parameter key.
- If the field has a default value (or default_factory), the parameter is optional and the default is used when the key 
is not supplied in YAML.
- If the field has no default, the parameter is required; `load()` will raise if it is missing / unset.
- Nested dataclasses are supported and map to nested parameter dictionaries.

For example, you can create the following config dataclass:
```py
from dataclasses import dataclass, field

@dataclass
class Weights:
    heading: float = 1.0
    clearance: float

@dataclass
class PlannerConfig:
    max_iters: int = 10_000
    timeout_s: float
    weights: Weights
```

Which would map to a ROS2 config yaml structure like this:
```yaml
planner:
  ros__parameters:
    # max_iters is optional (defaults to 10000)
    timeout_s: 3.0            # required
    weights:
      # heading is optional (defaults to 1.0)
      clearance: 0.75         # required
```

You can then load it in code by calling nav_utils.config.load
```py
from rclpy.node import Node
import nav_utils.config

class Planner(Node):
    def __init__(self):
        self.config = nav_utils.config.load(self, PlannerConfig)
        print(self.config.max_iters)
        print(self.config.timeout_s)
        print(self.config.weights.clearance)
        print(self.config.weights.heading)
```

## nav_utils.geometry
2D geometry helpers for ROS2 message types.

| Function | Description |
|---|---|
| `point_is_close(a, b)` | True if two points are within 1cm on all axes |
| `distance(a, b)` | 2D Euclidean distance between two points (z ignored) |
| `rotate_by_yaw(point, angle)` | Rotate a point about +Z by an angle in radians |
| `get_yaw_radians_from_quaternion(q)` | Extract yaw in radians from a ROS quaternion |
| `make_quaternion_from_yaw(yaw)` | Construct a pure-yaw ROS quaternion |
| `make_pose(x, y, yaw)` | Construct a 2D `Pose` from position and heading |

## nav_utils.world_occupancy_grid
This class provides a world-coordinate view of a discrete, robot-centric occupancy grid. 

It allows planners to operate entirely on world `Point`s—querying occupancy, expanding neighbors, and hashing 
locations—without directly interacting with grid indices. Conceptually, the occupancy grid is treated as an infinite 
world representation: world points are projected into grid cells on demand, and any point outside the underlying grid 
bounds is treated as unknown.

### Conventions / Transformations
The supplied occupancy grid is assumed to have the following conventions (matching ROS conventions):
- +x points forward from the robot
- +y points to the left of the robot
- the grid origin is the bottom-left corner of the grid
- data is row major

### State
State of the occupancy grid at some point can be queried using `state(point)`, which returns a `CellState`. Points
outside `[0..width) × [0..height)` return an unknown cell.

`CellState` exposes the following properties:

| Property | Description |
|---|---|
| `is_unknown` | True if the cell lies outside the grid bounds |
| `is_drivable` | True if occupancy probability is ≤ 30, or the cell is a self-drive goal |
| `is_self_drive_goal` | True if the cell is marked as a self-drive goal (value `127`) |

### Full grid iteration in continuous space via in_bound_points
To iterate through all in bound grids, WorldOccupancyGrid provides `in_bound_points`. This iterates through every grid 
in the occupancy grid and yields the center of the grid. 

Example pattern:
```py
for candidate in grid.in_bound_points():
    if grid.state(candidate).is_self_drive_goal:
        # found self drive goal, do something special
```

### Discrete “search” in continuous space via neighbors
Although planner code operates on continuous world `Point`s, discrete graph search can still be performed using 
`neighbors4(point)`, `neighbors8(point)`, or `neighbors_forward(point)`.

Each neighbor expansion:
1. Projects the input world point into a grid cell
2. Expands neighboring cells in grid index space
3. Converts those neighboring cells back into world points by returning the center of each cell

As a result, the search is discrete in the occupancy grid, while planner logic remains entirely in world coordinates.

Example pattern:
```py
for candidate in grid.neighbors8(current):
    if not grid.state(candidate).is_drivable:
        continue
```
### Hashing
To support discrete search bookkeeping (e.g. visited sets), `WorldOccupancyGrid` provides `hash_key(point)`, which 
returns a stable integer identifier corresponding to the grid cell that the point belongs to.

The hash is derived from the projected grid indices, ensuring that all world points falling within the same grid cell 
map to the same key. This allows planners to treat grid cells as discrete states without storing raw grid indices or 
floating-point coordinates.
