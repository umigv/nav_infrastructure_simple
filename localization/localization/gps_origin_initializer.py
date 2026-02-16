import math
from statistics import median

import nav_utils.config
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from robot_localization.srv import SetDatum
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Empty

from .gps_origin_initializer_config import GpsOriginInitializerConfig


class GpsOriginInitializer(Node):
    def __init__(self):
        super().__init__("gps_origin_initializer")

        self.config = nav_utils.config.load(self, GpsOriginInitializerConfig)

        self.gps_subscriber = self.create_subscription(NavSatFix, "gps", self.gps_callback, 10)
        self.get_logger().info(f"Subscribing to GPS data on topic: {self.gps_subscriber.topic_name}")

        self.client = self.create_client(SetDatum, "set_datum")
        self.get_logger().info(f"Waiting for service {self.client.srv_name} to be available...")
        self.client.wait_for_service()

        self._localization_init_publisher = self.create_publisher(
            Empty,
            "localization_initialized",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        self.samples: list[NavSatFix] = []
        self.sent = False

        self.get_logger().info(
            f"Collecting GNSS samples for datum. Keep robot STILL.\n"
            f"Policy: min_samples={self.config.min_samples_required}, max_sigma={self.config.max_h_sigma_m}m,\n"
            f"        min_duration={self.config.min_sample_duration_sec}s, max_duration={self.config.max_sample_duration_sec}s\n"
        )

    def gps_callback(self, msg: NavSatFix):
        if self.sent:
            return

        self.get_logger().debug(f"Received data: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}")

        h_sigma = math.sqrt(msg.position_covariance[0])
        if h_sigma > self.config.max_h_sigma_m:
            self.get_logger().debug(
                f"Dropping GPS msg with high horizontal sigma: {h_sigma} > {self.config.max_h_sigma_m}"
            )
            return

        self.samples.append(msg)

        time_elapsed = (self.samples[-1].header.stamp.sec - self.samples[0].header.stamp.sec) + (
            self.samples[-1].header.stamp.nanosec - self.samples[0].header.stamp.nanosec
        ) / 1e9
        if (
            len(self.samples) >= self.config.min_samples_required
            and time_elapsed >= self.config.min_sample_duration_sec
        ):
            self.get_logger().info("Sufficient samples collected.")
            self.send_origin_request()
        elif time_elapsed >= self.config.max_sample_duration_sec:
            self.get_logger().info("Max sample duration reached.")
            self.send_origin_request()

    def send_origin_request(self):
        self.get_logger().info(
            f"Collected {len(self.samples)} samples below horizontal sigma {self.config.max_h_sigma_m}m."
        )
        self.get_logger().info(str([(sample.latitude, sample.longitude) for sample in self.samples]))
        latitude = median(sample.latitude for sample in self.samples)
        longitude = median(sample.longitude for sample in self.samples)

        self.get_logger().info(f"Setting origin to median lat={latitude:.8f}, lon={longitude:.8f}")
        request = SetDatum.Request()
        request.geo_pose.position.latitude = latitude
        request.geo_pose.position.longitude = longitude
        request.geo_pose.position.altitude = 0.0
        # The GPS doesn't supply rotation information. Since this is used to initialize the GPS origin and we
        # make (0, 0) the starting point of the robot, the rotation at the geo pose is 0.
        request.geo_pose.orientation.w = 1.0
        request.geo_pose.orientation.x = 0.0
        request.geo_pose.orientation.y = 0.0
        request.geo_pose.orientation.z = 0.0

        self.sent = True
        self.destroy_subscription(self.gps_subscriber)

        fut = self.client.call_async(request)
        fut.add_done_callback(self.on_response)

    def on_response(self, future):
        try:
            future.result()
            self._localization_init_publisher.publish(Empty())
            self.get_logger().info("Origin set successfully. map frame is now aligned to startup GPS origin.")
        except Exception as e:
            self.get_logger().error(f"{self.client.srv_name} call error: {e}")


def main():
    rclpy.init()
    node = GpsOriginInitializer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
