import math
import random

import nav_utils.config
import rclpy
import tf2_geometry_msgs  # noqa: F401 - registers PointStamped transform handlers
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    Twist,
    TwistWithCovariance,
    TwistWithCovarianceStamped,
    Vector3,
)
from nav_msgs.msg import Odometry
from nav_utils.geometry import get_yaw_radians_from_quaternion, make_quaternion_from_yaw, rotate_by_yaw
from pyproj import Transformer
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from .sensor_simulator_config import SensorSimulatorConfig


class SensorSimulator(Node):
    def __init__(self) -> None:
        super().__init__("sensor_simulator")
        self.config: SensorSimulatorConfig = nav_utils.config.load(self, SensorSimulatorConfig)

        enu_to_lla = f"""
            +proj=pipeline
            +step +inv +proj=topocentric
                +lat_0={self.config.gps.origin_latitude_deg}
                +lon_0={self.config.gps.origin_longitude_deg}
                +h_0=0
                +ellps=WGS84
            +step +inv +proj=cart +ellps=WGS84
        """

        self.from_enu = Transformer.from_pipeline(enu_to_lla)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

        self.enc_vel_publisher = self.create_publisher(TwistWithCovarianceStamped, "enc_vel", 10)
        self.imu_publisher = self.create_publisher(Imu, "imu", 10)
        self.gps_publisher = self.create_publisher(NavSatFix, "gps", 10)
        self.ground_truth_publisher = self.create_publisher(Odometry, "ground_truth", 10)

        self.x: float = 0.0
        self.y: float = 0.0
        self.yaw: float = self.config.imu.initial_yaw_rad
        self.cmd_vel = Twist()
        self.last_cmd_time = self.get_clock().now()

        self.enc_vx_scale: float = 0.0
        self.enc_wz_scale: float = 0.0
        self.gps_dx: float = 0.0
        self.gps_dy: float = 0.0

        self.create_timer(self.config.high_rate_update_period_s, self.high_rate_update)
        self.create_timer(self.config.low_rate_update_period_s, self.low_rate_update)

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.cmd_vel = msg
        self.last_cmd_time = self.get_clock().now()

    def high_rate_update(self) -> None:
        now = self.get_clock().now()
        if (now - self.last_cmd_time) > Duration(seconds=self.config.cmd_vel_timeout_s):
            self.cmd_vel = Twist()

        self.x += self.cmd_vel.linear.x * self.config.high_rate_update_period_s * math.cos(self.yaw)
        self.y += self.cmd_vel.linear.x * self.config.high_rate_update_period_s * math.sin(self.yaw)
        self.yaw += self.cmd_vel.angular.z * self.config.high_rate_update_period_s
        self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi  # wrap to [-pi, pi]

        self.publish_ground_truth()
        self.publish_enc_vel()
        self.publish_imu()

    def low_rate_update(self) -> None:
        self.publish_gps()

    def publish_ground_truth(self) -> None:
        self.ground_truth_publisher.publish(
            Odometry(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.config.map_frame_id),
                child_frame_id=self.config.base_frame_id,
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(x=self.x, y=self.y, z=0.0),
                        orientation=make_quaternion_from_yaw(self.yaw),
                    )
                ),
                twist=TwistWithCovariance(twist=self.cmd_vel),
            )
        )

    def publish_enc_vel(self) -> None:
        self.enc_vx_scale = self.ou_update(
            self.enc_vx_scale,
            self.config.enc_vel.vx_drift_std,
            self.config.enc_vel.drift_time_constant_s,
            self.config.high_rate_update_period_s,
        )

        self.enc_wz_scale = self.ou_update(
            self.enc_wz_scale,
            self.config.enc_vel.wz_drift_std,
            self.config.enc_vel.drift_time_constant_s,
            self.config.high_rate_update_period_s,
        )

        measurement_vx = self.cmd_vel.linear.x * (1.0 + self.enc_vx_scale) + random.gauss(
            0.0, self.config.enc_vel.vx_noise_std_mps
        )
        measurement_wz = self.cmd_vel.angular.z * (1.0 + self.enc_wz_scale) + random.gauss(
            0.0, self.config.enc_vel.wz_noise_std_radps
        )

        # fmt: off
        enc_vel_cov = [
            self.config.enc_vel.vx_noise_std_mps**2, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, self.config.enc_vel.wz_noise_std_radps**2,
        ]
        # fmt: on

        self.enc_vel_publisher.publish(
            TwistWithCovarianceStamped(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.config.base_frame_id),
                twist=TwistWithCovariance(
                    twist=Twist(
                        linear=Vector3(x=measurement_vx),
                        angular=Vector3(z=measurement_wz),
                    ),
                    covariance=enc_vel_cov,
                ),
            )
        )

    def publish_imu(self) -> None:
        try:
            yaw_offset = get_yaw_radians_from_quaternion(
                self.tf_buffer.lookup_transform(
                    self.config.base_frame_id, self.config.imu_frame_id, self.get_clock().now()
                ).transform.rotation
            )
        except TransformException as e:
            self.get_logger().warn(
                f"TF {self.config.base_frame_id}->{self.config.imu_frame_id} unavailable, skipping publishing IMU: {e}"
            )
            return

        imu_mag_yaw = self.yaw + yaw_offset - self.config.imu.magnetic_declination_rad
        measurement_yaw = imu_mag_yaw + random.gauss(0.0, self.config.imu.yaw_noise_std_rad)
        measurement_wz = self.cmd_vel.angular.z + random.gauss(0.0, self.config.imu.wz_noise_std_radps)

        orientation_var = self.config.imu.yaw_noise_std_rad**2
        angular_velocity_var = self.config.imu.wz_noise_std_radps**2

        # fmt: off
        orientation_cov = [
            orientation_var, 0.0,              0.0,
            0.0,             orientation_var,  0.0,
            0.0,             0.0,              orientation_var,
        ]
        angular_velocity_cov = [
            angular_velocity_var, 0.0,                  0.0,
            0.0,                  angular_velocity_var, 0.0,
            0.0,                  0.0,                  angular_velocity_var,
        ]
        linear_acceleration_cov = [
            -1.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
        ]
        # fmt: on

        self.imu_publisher.publish(
            Imu(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.config.imu_frame_id),
                orientation=make_quaternion_from_yaw(measurement_yaw),
                orientation_covariance=orientation_cov,
                angular_velocity=Vector3(z=measurement_wz),
                angular_velocity_covariance=angular_velocity_cov,
                linear_acceleration_covariance=linear_acceleration_cov,
            )
        )

    def publish_gps(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.config.base_frame_id, self.config.gps_frame_id, self.get_clock().now()
            )
            transform = Point(x=tf.transform.translation.x, y=tf.transform.translation.y)
            rotated = rotate_by_yaw(transform, self.yaw)
            gps_position = Point(x=self.x + rotated.x, y=self.y + rotated.y)
        except TransformException as e:
            self.get_logger().warn(
                f"TF {self.config.base_frame_id}->{self.config.gps_frame_id} unavailable, skipping publishing GPS: {e}"
            )
            return

        self.gps_dx = self.ou_update(
            self.gps_dx,
            self.config.gps.drift_std_m,
            self.config.gps.drift_time_constant_s,
            self.config.low_rate_update_period_s,
        )

        self.gps_dy = self.ou_update(
            self.gps_dy,
            self.config.gps.drift_std_m,
            self.config.gps.drift_time_constant_s,
            self.config.low_rate_update_period_s,
        )

        measurement_x = gps_position.x + self.gps_dx + random.gauss(0.0, self.config.gps.noise_std_m)
        measurement_y = gps_position.y + self.gps_dy + random.gauss(0.0, self.config.gps.noise_std_m)
        lon, lat, _ = self.from_enu.transform(measurement_x, measurement_y, 0.0)

        # fmt: off
        gps_cov = [
            self.config.gps.sigma2,  0.0, 0.0,
            0.0, self.config.gps.sigma2,  0.0,
            0.0, 0.0, self.config.gps.sigma2,
        ]
        # fmt: on

        self.gps_publisher.publish(
            NavSatFix(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.config.gps_frame_id),
                status=NavSatStatus(status=NavSatStatus.STATUS_FIX, service=NavSatStatus.SERVICE_GPS),
                latitude=lat,
                longitude=lon,
                altitude=0.0,
                position_covariance=gps_cov,
                position_covariance_type=NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN,
            )
        )

    @staticmethod
    def ou_update(value: float, sigma: float, tau: float, dt: float) -> float:
        """Does one iteration of Ornstein-Uhlenbeck process.
        https://en.wikipedia.org/wiki/Ornstein%E2%80%93Uhlenbeck_process
  
        Args:
            value: Current state x_t.
            sigma: Steady-state standard deviation. Units match value.
            tau: Mean-reversion time constant in seconds. Larger values produce slower-drifting, more correlated noise.
            dt: Elapsed time in seconds. If <= 0, value is returned unchanged.

        Returns:
            Updated state.
        """
        if dt <= 0.0:
            return value

        decay = math.exp(-dt / tau)
        innov_std = sigma * math.sqrt(max(0.0, 1.0 - decay**2))
        return value * decay + innov_std * random.gauss(0.0, 1.0)


def main() -> None:
    rclpy.init()
    node = SensorSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
