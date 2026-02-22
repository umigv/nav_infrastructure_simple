# localization
Localization helper nodes for the navigation stack. See `nav_bringup/launch/localization.launch.py` for how these are 
launched together.

---

## gps_origin_initializer
Collects GPS samples on startup, computes the median position, and calls the `robot_localization` `set_datum` service to 
set the map origin. Publishes on `localization_initialized` when done.

Samples are filtered by horizontal accuracy (`max_horizontal_stdev_m`). The node waits until either `min_samples_required` 
samples have been collected for at least `min_sample_duration_s`, or `max_sample_duration_s` has elapsed.

### Subscribed Topics
- `gps` (`sensor_msgs/NavSatFix`) - GPS fix

### Published Topics
- `localization_initialized` (`std_msgs/Empty`) - initialized if datum is set

### Service Clients
- `set_datum` (`robot_localization/SetDatum`) - sets datum of the global odom node

### Config Parameters
| Parameter | Default | Description |
|---|---|---|
| `min_samples_required` | `100` | Minimum number of valid GPS samples before setting datum |
| `min_sample_duration_s` | `5.0` | Minimum time (s) to collect samples |
| `max_sample_duration_s` | `30.0` | Maximum time (s) before setting datum regardless of sample count |
| `max_horizontal_stdev_m` | `1.0` | Maximum horizontal accuracy (m) for a sample to be accepted |

---

## enc_odom_publisher
Integrates wheel encoder velocity to produce odometry using unicycle kinematics. Publishes an `Odometry` message and 
broadcasts the `odom → base_link` TF transform.

Intended as a drop-in replacement for `ekf_local` when IMU-corrected local odometry is not needed or desired. The global 
EKF (`ekf_global`) can still fuse this with GPS and absolute IMU for a map-frame estimate.

### Subscribed Topics
- `enc_vel` (`geometry_msgs/TwistWithCovarianceStamped`) - encoder velocity

### Published Topics
- `odom` (`nav_msgs/Odometry`) - integrated pose and velocity

### TF Broadcasts
- `odom → base_link` - derived from `odom_frame_id` and `base_frame_id` config

### Config Parameters
| Parameter | Default | Description |
|---|---|---|
| `pose_x_variance_m2` | `0.01` | Pose covariance for x (m²) |
| `pose_y_variance_m2` | `0.01` | Pose covariance for y (m²) |
| `pose_yaw_variance_rad2` | `0.01` | Pose covariance for yaw (rad²) |
| `odom_frame_id` | `odom` | Parent frame for odometry and TF |
| `base_frame_id` | `base_link` | Child frame for odometry and TF |

Twist covariance is propagated directly from the encoder driver message. Pose covariance is fixed diagonal. Tune these 
based on your encoder's expected drift characteristics.
