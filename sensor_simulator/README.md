# Sensor Simulator
Simulates encoder velocity, IMU, and GPS sensors for testing the localization stack without real hardware.
Subscribes to `cmd_vel`, integrates velocity into a ground-truth pose, and publishes noisy sensor measurements that
mimic real hardware behavior.

Sensor noise is modeled using two components: per-sample **Gaussian white noise** for instantaneous measurement error,
and [**Ornstein-Uhlenbeck (OU) processes**](https://en.wikipedia.org/wiki/Ornstein%E2%80%93Uhlenbeck_process) for 
slow-drifting correlated errors (e.g. scale factor bias on encoders, multipath drift on GPS). OU processes are 
mean-reverting, so errors stay bounded over time rather than growing without bound like a random walk.

## Behavior
- Integrates `cmd_vel` using 2D unicycle kinematics (Euler integration) at `high_rate_update_period_s`
- Velocity is zeroed if no `cmd_vel` is received within `cmd_vel_timeout_s`
- Robot starts at map position `(0, 0)` with heading `imu.initial_yaw_rad`
- All coordinates are ENU (East-North-Up): yaw=0 faces east, positive yaw is counter-clockwise

## Sensor Conventions

### Encoder Velocity
- Publishes `TwistWithCovarianceStamped` in the `base_link` frame at `high_rate_update_period_s`
- Applies Gaussian white noise to both linear and angular velocity
- Applies OU multiplicative scale-factor drift to both linear and angular velocity - the bias is proportional to speed,
so a stationary robot always reads near zero and odom drift only accumulates while moving

### IMU
- Publishes `Imu` in the `imu_link` frame at `high_rate_update_period_s`
- Simulates a 9-DOF magnetometer IMU that produces absolute orientation in the ENU frame (e.g. VectorNav VN-100)
- Yaw is reported relative to **magnetic north** (as a real magnetometer would), so the consumer is expected to apply
magnetic declination correction
- Only Gaussian white noise is applied - no drift, as absolute orientation sensors don't accumulate heading error

### GPS
- Publishes `NavSatFix` in the `gps_link` frame at `low_rate_update_period_s`
- The true GPS antenna position is computed from the robot pose using the `base_link` → `gps_link` TF
- Applies Gaussian white noise to simulate per-fix position error
- Applies independent OU position drift in X and Y in the map frame, modeling slow-varying correlated errors such as
multipath and ionospheric delay - drift is bounded (mean-reverting) unlike a random walk
- ENU map-frame position is converted to WGS84 lat/lon using a local topocentric projection anchored at
`(origin_latitude_deg, origin_longitude_deg)`

## Subscribed Topics
- `cmd_vel` (`geometry_msgs/Twist`) - Velocity commands

## Published Topics
- `enc_vel` (`geometry_msgs/TwistWithCovarianceStamped`) - Simulated encoder velocity in `base_link` frame
- `imu` (`sensor_msgs/Imu`) - Simulated IMU in `imu_link` frame
- `gps` (`sensor_msgs/NavSatFix`) - Simulated GPS fix in `gps_link` frame
- `ground_truth` (`nav_msgs/Odometry`) - Noiseless true pose in `map` frame

## Required TF
- `base_link` → `imu_link` - IMU mounting orientation (used to compute yaw offset)
- `base_link` → `gps_link` - GPS antenna position relative to the robot center

## Config Parameters

### Top-level
| Parameter | Type | Default | Description |
|---|---|---|---|
| `map_frame_id` | `str` | `map` | TF frame ID for the map frame |
| `base_frame_id` | `str` | `base_link` | TF frame ID for the robot base frame |
| `imu_frame_id` | `str` | `imu_link` | TF frame ID for the IMU |
| `gps_frame_id` | `str` | `gps_link` | TF frame ID for the GPS |
| `cmd_vel_timeout_s` | `float` | `0.5` | Seconds without a cmd_vel before velocity is zeroed |
| `high_rate_update_period_s` | `float` | `0.01` | Publish period for `enc_vel` and `imu` (s) |
| `low_rate_update_period_s` | `float` | `0.1` | Publish period for `gps` (s) |

### `enc_vel`
| Parameter | Type | Default | Description |
|---|---|---|---|
| `vx_noise_std_mps` | `float` | `0.0` | Per-sample Gaussian noise std on linear.x (m/s) |
| `wz_noise_std_radps` | `float` | `0.0` | Per-sample Gaussian noise std on angular.z (rad/s) |
| `vx_drift_std` | `float` | `0.0` | OU scale-factor steady-state std on linear.x |
| `wz_drift_std` | `float` | `0.0` | OU scale-factor steady-state std on angular.z |
| `drift_time_constant_s` | `float` | `30.0` | OU mean-reversion time constant for both scale factors (s) |

### `imu`
| Parameter | Type | Default | Description |
|---|---|---|---|
| `yaw_noise_std_rad` | `float` | `0.0` | Per-sample Gaussian noise std on absolute yaw (rad) |
| `wz_noise_std_radps` | `float` | `0.0` | Per-sample Gaussian noise std on angular velocity z (rad/s) |
| `magnetic_declination_rad` | `float` | `0.0` | Magnetic declination at the site (rad, east-positive). Shifts IMU output from true-north to magnetic-north frame. |
| `initial_yaw_rad` | `float` | `0.0` | Initial robot heading in true-north ENU map frame (rad) |

### `gps`
| Parameter | Type | Default | Description |
|---|---|---|---|
| `origin_latitude_deg` | `float` | required | WGS84 latitude of map origin. Map `(0, 0)` maps to this point. |
| `origin_longitude_deg` | `float` | required | WGS84 longitude of map origin. |
| `noise_std_m` | `float` | `0.0` | Per-sample Gaussian noise std on horizontal position (m) |
| `drift_std_m` | `float` | `0.0` | OU position-drift steady-state std (m). Set to `0` to disable drift. |
| `drift_time_constant_s` | `float` | `60.0` | OU mean-reversion time constant for position drift (s) |
