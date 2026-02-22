# nav_bringup
Launch files and shared configuration for the navigation stack. Frame IDs and other system-wide constants are defined
in `nav_bringup/global_config.py`.


## core.launch.py
Launches core functionalities used across the rest of the launch files

```
ros2 launch nav_bringup core.launch.py
```

### Subscribed Topics
- `teleop_cmd_vel` (`geometry_msgs/Twist`) - Joystick velocity
- `recovery_cmd_vel` (`geometry_msgs/Twist`) - Recovery velocity
- `nav_cmd_vel` (`geometry_msgs/Twist`) - Nav velocity

### Published Topics
- `cmd_vel` (`geometry_msgs/Twist`) - Multiplexed output velocity
- `state` (`std_msgs/msg/String`) - State (`normal`, `ramp` or `recovery`)

### Services
- `state/set_recovery` (`std_srvs/SetBool`) - Set whether we are in recovery mode
- `state/set_ramp` (`std_srvs/SetBool`) - Set whether we are in ramp mode

### Velocity Multiplexing
| Priority | Topic | Source | Timeout |
|---|---|---|---|
| 3 | `teleop_cmd_vel` | Joystick | 0.5s |
| 2 | `recovery_cmd_vel` | Recovery system | 0.5s |
| 1 | `nav_cmd_vel` | Autonomy | 0.5s |

If a higher-priority source stops publishing, control falls back to the next source after 0.5s.


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
Launches localization. In simulation mode, runs the localization simulator in place of real localization nodes . On real
hardware, either `ekf_local` or `enc_odom_publisher` is used for local odometry depending on `use_enc_odom`.

```
ros2 launch nav_bringup localization.launch.py [simulation:=true] [use_enc_odom:=true]
```

### Parameters
- `simulation`: Run the localization simulator instead of real localization, default `false`
- `use_enc_odom`: Use encoder odometry integration instead of EKF for local odometry, default `false`

### Subscribed Topics (real hardware, `simulation:=false`)
- `imu/raw` (`sensor_msgs/Imu`) - Raw IMU data
- `gps/raw` (`sensor_msgs/NavSatFix`) - Raw GPS fix
- `enc_vel/raw` (`geometry_msgs/TwistWithCovarianceStamped`) - Encoder velocity (only when `use_enc_odom:=true`)

### Subscribed Topics (simulation, `simulation:=true`)
- `cmd_vel` (`geometry_msgs/Twist`) - Velocity command used to integrate simulated robot position

### Published Topics
- `odom/local` (`nav_msgs/Odometry`) - Local odometry in the odom frame
- `odom/global` (`nav_msgs/Odometry`) - Global odometry in the map frame
- `localization_initialized` (`std_msgs/Empty`) - Latched, published once GPS origin is set (real hardware only)

### Broadcasted TF Frames
- `odom` → `base_link`
- `map` → `odom`

### Services
- `fromLL` (`robot_localization/FromLL`) - Converts GPS latitude/longitude to a map-frame point


## navigation.launch.py
Launches the navigation stack.

```
ros2 launch nav_bringup navigation.launch.py
```


## teleop.launch.py
Launches joystick teleoperation

```
ros2 launch nav_bringup teleop.launch.py controller:=<controller>
```

### Parameters
- `controller`: Controller profile (`xbox` or `ps4`), required
- `joystick_dev`: Joystick device path, default `/dev/input/js0`

### Controller Mappings
For both Xbox and PS4:
- Left joystick - linear motion
- Right joystick - turning
- Right shoulder button (RB / R1) - enable
- Left shoulder button (LB / L1) - turbo

### Published Topics
- `joy` (`sensor_msgs/Joy`) - Raw joystick input
- `teleop_cmd_vel` (`geometry_msgs/Twist`) - Joystick velocity command
