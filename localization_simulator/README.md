# Localization Simulator
Simulated localization node for testing the navigation stack without real hardware. Subscribes to `cmd_vel`, integrates 
velocity into a pose, and publishes odometry + TF frames that mimic what a real localization system would produce.

## Behavior
- Integrates `cmd_vel` using simple 2D kinematics (unicycle model) at a fixed period
- Velocity is zeroed if no `cmd_vel` is received within the configurable timeout
- The UTM zone is automatically derived from the gps origin
- The `map` -> `odom` transform is identity

## Subscribed Topics
- `cmd_vel` (`geometry_msgs/Twist`) - Velocity commands from twist_mux

## Published Topics
- `/odom/local` (`nav_msgs/Odometry`) - Local odometry in the `odom` frame
- `/odom/global` (`nav_msgs/Odometry`) - Global odometry in the `map` frame

## TF Broadcasts
- `odom` → `base_link` - Robot pose from integrated cmd_vel
- `map` → `odom` - Identity

## Services
- `fromLL` (`robot_localization/srv/FromLL`) - Converts GPS lat/lon to map-frame coordinates relative to the GPS origin

## Config Parameters
| Parameter | Type | Default | Description |
|---|---|---|---|
| `gps_origin_latitude` | `float` | required | GPS origin latitude in WGS84 degrees. All map-frame coordinates are relative to this point. |
| `gps_origin_longitude` | `float` | required | GPS origin longitude in WGS84 degrees. All map-frame coordinates are relative to this point. |
| `update_period_s` | `float` | `0.01` | Period of the position integration and TF publishing timer (s). |
| `cmd_vel_timeout_s` | `float` | `0.5` | If no cmd_vel is received within this window the robot velocity is zeroed (s). |
| `map_frame_id` | `str` | `map` | TF frame ID for the map frame. |
| `odom_frame_id` | `str` | `odom` | TF frame ID for the odometry frame. |
| `base_frame_id` | `str` | `base_link` | TF frame ID for the robot base frame. |
