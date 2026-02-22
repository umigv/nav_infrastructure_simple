# nav_bringup
Launch files and shared configuration for the navigation stack. Frame IDs and other system-wide constants are defined
in `nav_bringup/global_config.py`.

## sensors.launch.py
Launches hardware sensor drivers and static TF transforms.

```
ros2 launch nav_bringup sensors.launch.py
```

### Published Topics
- `imu/raw` (`sensor_msgs/Imu`) - Raw IMU data
- `gps/raw` (`sensor_msgs/NavSatFix`) - Raw GPS fix

### Broadcasted TF Frames
- `base_link` → `imu_link`
- `base_link` → `gps_link`

## localization.launch.py
Launches localization. In simulation mode, runs the localization simulator in place of real localization hardware.

```
ros2 launch nav_bringup localization.launch.py [simulation:=true]
```

### Parameters
- `simulation`: Run the localization simulator instead of real localization, default `false`

### Subscribed Topics (simulation)
- `cmd_vel` (`geometry_msgs/Twist`) - Velocity command used to integrate simulated robot position

### Published Topics
- `odom/local` (`nav_msgs/Odometry`) - Odometry in the odom frame
- `odom/global` (`nav_msgs/Odometry`) - Odometry in the map frame

### Broadcasted TF Frames
- `odom` → `base_link`
- `map` → `odom`

### Services
- `fromLL` (`robot_localization/FromLL`) - Converts GPS latitude/longitude to a map-frame point

## infra.launch.py
Launches the navigation stack.

```
ros2 launch nav_bringup infra.launch.py [simulation:=true]
```

## teleop.launch.py
Launches joystick teleoperation and velocity multiplexing. twist_mux arbitrates between teleop, recovery, and autonomy
velocity sources, with teleop taking highest priority.

```
ros2 launch nav_bringup teleop.launch.py controller:=<controller>
```

### Parameters
- `controller`: Controller profile (`xbox` or `ps4`), required
- `joystick_dev`: Joystick device path, default `/dev/input/js0`

### Controller Mappings
For both Xbox and PS4:
- Left joystick — linear motion
- Right joystick — turning
- Right shoulder button (RB / R1) — enable
- Left shoulder button (LB / L1) — turbo

### Subscribed Topics
- `recovery_cmd_vel` (`geometry_msgs/Twist`) - Recovery velocity
- `nav_cmd_vel` (`geometry_msgs/Twist`) - Autonomy velocity

### Published Topics
- `joy` (`sensor_msgs/Joy`) - Raw joystick input
- `teleop_cmd_vel` (`geometry_msgs/Twist`) - Joystick velocity command
- `cmd_vel` (`geometry_msgs/Twist`) - Multiplexed output velocity

### Velocity Multiplexing
| Priority | Topic | Source | Timeout |
|---|---|---|---|
| 3 | `teleop_cmd_vel` | Joystick | 0.5s |
| 2 | `recovery_cmd_vel` | Recovery system | 0.5s |
| 1 | `nav_cmd_vel` | Autonomy | 0.5s |

If a higher-priority source stops publishing, control falls back to the next source after 0.5s.
