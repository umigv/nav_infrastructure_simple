# occupancy_grid_transform
This package consumes the occupancy grid provided by CV, adds a border on the far edges, applies obstacle inflation, and 
republishes the occupancy grid suitable for planning. The grid origin is transformed from the incoming frame to the 
configured output frame using TF2.

## Grid conventions
The occupancy grid from CV is expected to match the convention of 
[`nav_msgs/OccupancyGrid`](https://docs.ros2.org/foxyapi/nav_msgs/msg/OccupancyGrid.html)
- Row major
- +x is forward, +y is left of the robot
- Height = number of cells in +y direction, width = number of cells in +x direction
- info.origin is the bottom left corner of the occupancy grid in the given frame
- Indexed with (y, x) in 2d, and (y * width + x) in 1d.

## Subscribed Topics
- `occupancy_grid` (`nav_msgs/msg/OccupancyGrid`) - Input occupancy grid in CV frame

## Published Topics
- `transformed_occupancy_grid` (`nav_msgs/msg/OccupancyGrid`) - Bordered and inflated occupancy grid in the configured output frame

## Inflation
After the grid is converted and a border is added, obstacle inflation is applied. For each occupied cell, surrounding
cells are inflated based on their distance:
- Cells within `inflation_radius_cells` are set to fully occupied (100)
- Cells between `inflation_radius_cells` and `inflation_falloff_radius_cells` decay as `100 × decay^(dist - radius)`
- Cells beyond `inflation_falloff_radius_cells` are unaffected

## Config Parameters
| Parameter | Default | Description |
|---|---|---|
| `frame_id` | `odom` | TF frame the transformed grid is published in |

### Inflation Parameters (`inflation_params`)
| Parameter | Default | Description |
|---|---|---|
| `inflation_radius_cells` | `10` | Radius (cells) of the fully inflated obstacle core |
| `inflation_falloff_radius_cells` | `20` | Maximum radius (cells) of obstacle influence |
| `inflation_decay_factor` | `0.9` | Decay factor in the falloff region (0–1, higher = slower decay) |
