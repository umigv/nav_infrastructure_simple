# Localization Simulator
Simulated localization node for testing the navigation stack without real hardware. Subscribes to `/cmd_vel`, integrates velocity into a pose, and publishes odometry + TF frames that mimic what a real localization system would produce.

## Behavior
- Integrates `/cmd_vel` using simple 2D kinematics (unicycle model) at a fixed period
- Velocity is zeroed if no `/cmd_vel` is received within the configurable timeout
- The UTM zone is automatically derived from the gps origin
- The `map` -> `odom` transform is identity

## Subscribed Topics
`/cmd_vel` (`geometry_msgs/Twist`) - velocity commands from twist_mux

## Published Topics
`/odom/local` (`nav_msgs/Odometry`) - local odometry in the `odom` frame
`/odom/global` (`nav_msgs/Odometry`) - global odometry in the `map` frame

## TF Broadcasts
`odom` -> `base_link` - robot pose from integrated cmd_vel
`map` -> `odom` - identity

## Services
`fromLL` (`robot_localization/srv/FromLL`) - converts GPS lat/lon to map-frame coordinates relative to the gps origin
