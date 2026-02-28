# path_tracking
Path tracking for mobile robots using pure pursuit.

Subscribes to a planned path and odometry, fits a B-spline through the path waypoints, and publishes velocity commands using the pure pursuit algorithm. The lookahead distance adapts with current speed to improve tracking at higher velocities.

## Subscribed Topics
- `odom` (`nav_msgs/msg/Odometry`) - Robot pose and velocity in the odometry frame
- `path` (`nav_msgs/msg/Path`) - Planned path to follow

## Published Topics
- `nav_cmd_vel` (`geometry_msgs/msg/Twist`) - Velocity command for the robot base
- `smoothed_path` (`nav_msgs/msg/Path`) - B-spline smoothed version of the received path, for visualization

## Config Parameters
| Parameter | Type | Default | Description |
|---|---|---|---|
| `max_angular_speed_radps` | `float` | `0.6` | Maximum angular velocity command (rad/s). Commands are scaled down to stay within this limit. |
| `base_lookahead_distance_m` | `float` | `0.1` | Lookahead distance when stationary (m). Added to the speed-dependent term. |
| `min_lookahead_distance_m` | `float` | `0.1` | Minimum clamped lookahead distance (m). |
| `max_lookahead_distance_m` | `float` | `0.4` | Maximum clamped lookahead distance (m). |
| `lookahead_speed_gain` | `float` | `0.55` | Gain applied to current speed when computing adaptive lookahead distance. |
| `linear_speed_gain` | `float` | `2.0` | Gain applied to lookahead distance to produce the linear velocity command. |
| `spline_smoothing` | `float` | `0.1` | Smoothing factor passed to scipy `splprep`. Higher values smooth more aggressively. |
| `control_period_s` | `float` | `0.1` | Period of the control loop timer (s). |
| `base_frame_id` | `str` | `base_link` | Frame ID of the robot base, used to validate the child frame of incoming odometry. |
| `odom_frame_id` | `str` | `odom` | Frame ID of the odometry frame, used to validate incoming odometry and path messages. |
