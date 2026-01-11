from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped

class GpsOdomFusion(Node):
    def __init__(self):
        super().__init__('gps_odom_fusion')

        # Subscription
        self.odom_subscription = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.gps_subscription = self.create_subscription(NavSatFix, "/gps_coords", self.gps_callback, 10)

        # Publisher
        self.global_odom_publisher = self.create_publisher(Odometry, "/odom_global", 10)
        self.global_odom_publish_timer = self.create_timer(0.01, self.publish_global_odometry)

        # State
        self.gps_origin = None # long, lat
        self.global_odometry = None
        self.prev_odom_pose = None

    def odom_callback(self, msg: Odometry):
        pass

    def gps_callback(self, msg: NavSatFix):
        pass

    def publish_global_odometry(self):
        pass
