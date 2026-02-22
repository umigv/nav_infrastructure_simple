# path_planning
Path planning from the robot's position to a local goal using A*.

On each received goal, searches for the nearest drivable cell to the robot (via BFS, since the robot's position is 
typically in unknown space) and then runs A* through the occupancy grid to find a path to the goal. If the goal is 
unreachable, the path leads to the closest reachable cell. A linear bridge is interpolated from the robot's position to 
the first drivable cell and prepended to the path. The full path is then smoothed with a B-spline before publishing.

## Subscribed Topics
- `odom` (`nav_msgs/msg/Odometry`) — Robot pose in the planning frame
- `occupancy_grid` (`nav_msgs/msg/OccupancyGrid`) — Occupancy grid used for path planning
- `goal` (`geometry_msgs/msg/PointStamped`) — Local goal to plan towards; triggers replanning on each message

## Published Topics
- `path` (`nav_msgs/msg/Path`) — Planned path from the robot to the goal

## Config Parameters
| Parameter | Type | Default | Description |
|---|---|---|---|
| `max_search_radius_m` | `float` | `5.0` | Maximum distance (m) from the robot to search for a drivable starting point |
| `interpolation_resolution_m` | `float` | `0.05` | Distance (m) between interpolated points in the bridge from the robot to the first drivable cell |
| `spline_smoothing` | `float` | `0.1` | Smoothing factor passed to scipy `splprep`. Higher values smooth more aggressively. |
| `frame_id` | `str` | `odom` | TF frame ID for the occupancy grid, odometry, goal, and published path |
