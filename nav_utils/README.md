## nav_utils
Small utilities for ROS 2 Python nodes:

### nav_utils.config
Adds a simple config-loading utility for ROS 2.

To use, you will want to create a python dataclass where each field in the config dataclass corresponds to a ROS 2 parameter key. The field’s default value (if provided) is used as the parameter’s default.

Mapping rules:
- Each dataclass field name corresponds to a ROS 2 parameter key.
- If the field has a default value (or default_factory), the parameter is optional and the default is used when the key is not supplied in YAML.
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
    __init__(self):
        self.config = nav_utils.config.load(self, PlannerConfig)
        print(self.config.max_iters)
        print(self.config.timeout_s)
        print(self.config.weights.clearance)
        print(self.config.weights.heading)
```

### nav_utils.geometry
2D geometry helpers for ROS2 message types

### nav_utils.world_occupancy_grid
This class provides a world-coordinate view of a discrete, robot-centric occupancy grid. 

It allows planners to operate entirely on world `Point`s—querying occupancy, expanding neighbors, and hashing locations—without directly interacting with grid indices. Conceptually, the occupancy grid is treated as an infinite world representation:
world points are projected into grid cells on demand, and any point outside the underlying grid bounds is treated as `UNKNOWN`.

#### Conventions / Transformations
The supplied occupancy grid is assumed to have the following conventions (matching odom conventions):
- +x points forward from the robot
- +y points to the left of the robot
- the grid origin is the bottom-left corner of the grid
- data is row major

#### State
State of the occupancy grid at some point can be queried using `state(point)`. The state is treated as `CellState.UNKNOWN` if `(grid_x, grid_y)` lies outside `[0..width) × [0..height)`.

#### Discrete “search” in continuous space via neighbors
Although planner code operates on continuous world `Point`s, discrete graph search can still be performed using `neighbors4(point)`, `neighbors8(point)`, or `neighbors_forward(point)`.

Each neighbor expansion:
1. Projects the input world point into a grid cell
2. Expands neighboring cells in grid index space
3. Converts those neighboring cells back into world points by returning the center of each cell

As a result, the search is discrete in the occupancy grid, while planner logic remains entirely in world coordinates.

Example pattern:
```py
for candidate in grid.neighbors8(current):
    if not grid.state(candidate).drivable:
        continue
```
#### Hashing
To support discrete search bookkeeping (e.g. visited sets), `WorldOccupancyGrid` provides `hash_key(point)`, which returns a stable integer identifier corresponding to the grid cell that the point belongs to.

The hash is derived from the projected grid indices, ensuring that all world points falling within the same grid cell map to the same key. This allows planners to treat grid cells as discrete states without storing raw grid indices or floating-point coordinates.
