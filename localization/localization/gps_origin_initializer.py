import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from robot_localization.srv import SetDatum
from statistics import median

class GpsOriginInitializer(Node):
    def __init__(self):
        super().__init__("gps_origin_initializer")

        # Parameters
        self.declare_parameter("gps_data_topic", "/gps/raw")
        self.declare_parameter("set_origin_service", "/navsat_transform/set_datum")

        self.declare_parameter("min_samples_required", 100)
        self.declare_parameter("min_sample_duration_sec", 5.0)
        self.declare_parameter("max_sample_duration_sec", 30.0)
        self.declare_parameter("max_h_sigma_meter", 1.0)

        # Get Parameters
        self.gps_data_topic = self.get_parameter("gps_data_topic").value
        self.set_origin_service = self.get_parameter("set_origin_service").value

        self.min_samples_required = int(self.get_parameter("min_samples_required").value)
        self.min_sample_duration = float(self.get_parameter("min_sample_duration_sec").value)
        self.max_sample_duration = float(self.get_parameter("max_sample_duration_sec").value)
        self.max_h_sigma = float(self.get_parameter("max_h_sigma_meter").value)

        # Subscribers and Clients
        self.get_logger().info(f"Subscribing to GPS data on topic: {self.gps_data_topic}")
        self.gps_subscriber = self.create_subscription(NavSatFix, self.gps_data_topic, self.gps_callback, 10)

        self.get_logger().info(f"Waiting for service {self.set_origin_service} to be available...")
        self.client = self.create_client(SetDatum, self.set_origin_service)
        self.client.wait_for_service()

        self.samples: list[NavSatFix] = []
        self.sent = False

        self.get_logger().info(
            f"Collecting GNSS samples for datum. Keep robot STILL.\n"
            f"Policy: min_samples={self.min_samples_required}, max_sigma={self.max_h_sigma}m,\n"
            f"        min_duration={self.min_sample_duration}s, max_duration={self.max_sample_duration}s\n"
        )

    def gps_callback(self, msg: NavSatFix):
        if self.sent:
            return

        self.get_logger().debug(f"Received data: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}")

        h_sigma = msg.position_covariance[0] ** 0.5
        if h_sigma > self.max_h_sigma:
            self.get_logger().debug(f"Dropping GPS msg with high horizontal sigma: {h_sigma} > {self.max_h_sigma}")
            return

        self.samples.append(msg)

        time_elapsed = (self.samples[-1].header.stamp.sec - self.samples[0].header.stamp.sec) + \
                       (self.samples[-1].header.stamp.nanosec - self.samples[0].header.stamp.nanosec) / 1e9
        if len(self.samples) >= self.min_samples_required and time_elapsed >= self.min_sample_duration:
            self.get_logger().info("Sufficient samples collected.")
            self.send_origin_request()
        elif time_elapsed >= self.max_sample_duration:
            self.get_logger().info("Max sample duration reached.")
            self.send_origin_request()

    def send_origin_request(self):
        self.get_logger().info(f"Collected {len(self.samples)} samples below horizontal sigma {self.max_h_sigma}m.")
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
            self.get_logger().info("Origin set successfully. map frame is now aligned to startup GPS origin.")
        except Exception as e:
            self.get_logger().error(f"{self.set_origin_service} call error: {e}")
        
        self.destroy_node()

def main():
    rclpy.init()
    node = GpsOriginInitializer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
