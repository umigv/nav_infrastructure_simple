import nav_utils.config
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped transform handlers
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

from .occupancy_grid_transform_config import OccupancyGridTransformConfig
from .occupancy_grid_transform_impl import add_border, inflate_grid


class OccupancyGridTransform(Node):
    def __init__(self) -> None:
        super().__init__("occupancy_grid_transform")

        self.config: OccupancyGridTransformConfig = nav_utils.config.load(self, OccupancyGridTransformConfig)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, "occupancy_grid", self.occupancy_grid_callback, 10)

        self.publisher = self.create_publisher(OccupancyGrid, "transformed_occupancy_grid", 10)

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        try:
            origin_raw = PoseStamped(header=Header(frame_id=msg.header.frame_id), pose=msg.info.origin)
            origin_transformed = self.tf_buffer.transform(origin_raw, self.config.frame_id).pose
        except Exception as e:
            self.get_logger().error(f"TF {msg.header.frame_id}->{self.config.frame_id} unavailable, skipping: {e}")
            return

        grid = np.asarray(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        bordered_grid = add_border(grid)
        inflated_grid = inflate_grid(bordered_grid, self.config.inflation_params)
        height, width = grid.shape

        self.publisher.publish(
            OccupancyGrid(
                header=Header(frame_id=self.config.frame_id),
                info=MapMetaData(resolution=msg.info.resolution, width=width, height=height, origin=origin_transformed),
                data=inflated_grid.astype(np.int8).reshape(-1).tolist(),
            )
        )


def main() -> None:
    rclpy.init()
    node = OccupancyGridTransform()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
