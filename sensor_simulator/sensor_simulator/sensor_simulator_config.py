from dataclasses import dataclass


@dataclass(frozen=True)
class EncVelConfig:
    """Encoder velocity sensor configs.

    Measurement model::
        vx_meas = true_vx * (1 + vx_scale) + N(0, vx_noise_std_mps)
        wz_meas = true_wz * (1 + wz_scale) + N(0, wz_noise_std_radps)

    vx_scale / wz_scale are Ornstein-Uhlenbeck scale-factor biases. The bias is multiplicative, so a stationary robot
    always reads near zero. Odom-frame position drift only accumulates while the robot is moving.

    Attributes:
        vx_noise_std_mps: Per-sample Gaussian noise on linear.x.
        wz_noise_std_radps: Per-sample Gaussian noise on angular.z.
        vx_drift_std: OU scale-factor steady-state std on linear.x.
        wz_drift_std: OU scale-factor steady-state std on angular.z).
        drift_time_constant_s: OU mean-reversion time constant for both scale factors.
    """

    vx_noise_std_mps: float = 0.0
    wz_noise_std_radps: float = 0.0
    vx_drift_std: float = 0
    wz_drift_std: float = 0
    drift_time_constant_s: float = 30.0

    def __post_init__(self) -> None:
        if self.drift_time_constant_s <= 0:
            raise ValueError("EncVelConfig: drift_time_constant_s must be > 0")


@dataclass(frozen=True)
class ImuConfig:
    """IMU sensor configs.

    Measurement model::
        yaw_meas = true_yaw + imu_yaw_offset - magnetic_declination_rad + N(0, yaw_noise_std_rad)
        wz_meas = true_wz + N(0, wz_noise_std_radps)

    imu_yaw_offset is looked up from the imu_link→base_link TF. magnetic_declination shifts from true-north to 
    magnetic-north frame;

    Attributes:
        yaw_noise_std_rad: Per-sample Gaussian noise on absolute yaw (rad).
        wz_noise_std_radps: Per-sample Gaussian noise on angular velocity z (rad/s).
        magnetic_declination_rad: Magnetic declination at the deployment site (rad, west-negative)
        initial_yaw_rad: Initial heading of the robot in map frame in true north ENU (rad).
    """

    yaw_noise_std_rad: float = 0.0
    wz_noise_std_radps: float = 0.0
    magnetic_declination_rad: float = 0.0
    initial_yaw_rad: float = 0.0


@dataclass(frozen=True)
class GpsConfig:
    """GPS sensor configs.

    Measurement model::
        x_meas = (gps_x) + dx + N(0, noise_std_m)
        y_meas = (gps_y) + dy + N(0, noise_std_m)

    gps_{x,y} is the GPS antenna's true position transformed using gps_link→base_link TF. dx, dy are Ornstein-Uhlenbeck 
    position drift in map-frame. GPS drift is bounded (mean-reverting) unlike odom drift, modelling slow-varying 
    correlated errors such as multipath, ionospheric delay, and satellite geometry changes.

    Attributes:
        origin_latitude_deg: GPS origin latitude in WGS84. Map (0, 0) maps to this point.
        origin_longitude_deg: GPS origin longitude in WGS84.
        noise_std_m: Per-sample Gaussian noise on horizontal position (m).
        drift_std_m: OU position-drift steady-state std (m). Set to 0 to disable drift.
        drift_time_constant_s: OU mean-reversion time constant for position drift (s).
    """

    origin_latitude_deg: float
    origin_longitude_deg: float
    noise_std_m: float = 0.0
    drift_std_m: float = 0.0
    drift_time_constant_s: float = 60.0

    @property
    def sigma2(self) -> float:
        return self.noise_std_m**2 + self.drift_std_m**2

    def __post_init__(self) -> None:
        if self.drift_time_constant_s <= 0:
            raise ValueError("GpsConfig: drift_time_constant_s must be > 0")


@dataclass(frozen=True)
class SensorSimulatorConfig:
    """Top-level configuration for the sensor simulator node.

    Attributes:
        enc_vel: Encoder velocity sensor noise and drift config.
        imu: IMU sensor noise and magnetic declination config.
        gps: GPS sensor noise and drift config.
        map_frame_id: TF frame ID for the map frame.
        base_frame_id: TF frame ID for the robot base frame.
        imu_frame_id: TF frame ID for the IMU.
        gps_frame_id: TF frame ID for the GPS.
        cmd_vel_timeout_s: Seconds without a cmd_vel message before velocity is zeroed (s).
        high_rate_update_period_s: Publish period for enc_vel/raw and imu/raw (s).
        low_rate_update_period_s: Publish period for gps/raw (s).
    """

    enc_vel: EncVelConfig
    imu: ImuConfig
    gps: GpsConfig

    map_frame_id: str = "map"
    base_frame_id: str = "base_link"
    imu_frame_id: str = "imu_link"
    gps_frame_id: str = "gps_link"

    cmd_vel_timeout_s: float = 0.5

    high_rate_update_period_s: float = 0.01
    low_rate_update_period_s: float = 0.1

    def __post_init__(self) -> None:
        if self.cmd_vel_timeout_s <= 0:
            raise ValueError("SensorSimulatorConfig: cmd_vel_timeout_s must be > 0")
        if self.high_rate_update_period_s <= 0:
            raise ValueError("SensorSimulatorConfig: high_rate_update_period_s must be > 0")
        if self.low_rate_update_period_s <= 0:
            raise ValueError("SensorSimulatorConfig: low_rate_update_period_s must be > 0")
