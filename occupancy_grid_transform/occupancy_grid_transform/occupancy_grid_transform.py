import nav_utils.config
import numpy as np
import rclpy
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import Header

from .occupancy_grid_transform_impl import compute_origin_pose, cv_occupancy_grid_to_ros_grid, inflate_grid, weight_grid
from .occupancy_grid_trasform_config import OccupancyGridTransformConfig


class OccupancyGridTransform(Node):
    def __init__(self):
        super().__init__("occupancy_grid_transform")

        self.config: OccupancyGridTransformConfig = nav_utils.config.load(self, OccupancyGridTransformConfig)

        self._odom: Odometry | None = None

        self.create_subscription(OccupancyGrid, "occupancy_grid", self.occupancy_grid_callback, 10)
        self.create_subscription(Odometry, "odom", self.odom_callback, 10)

        self.publisher = self.create_publisher(OccupancyGrid, "occupancy_grid_transform", 10)

    def odom_callback(self, msg: Odometry) -> None:
        self._odom = msg

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        grid = cv_occupancy_grid_to_ros_grid(msg)
        inflated_grid = inflate_grid(grid, self.config.inflation_params)
        weighted_grid = weight_grid(inflated_grid, self.config.weight_params)
        height, width = grid.shape
        origin = compute_origin_pose(
            odom=self._odom.pose.pose if self._odom is not None else None,
            robot_forward_offset_m=self.config.robot_forward_offset_m,
            grid_height_cells=height,
            resolution=msg.info.resolution,
        )

        self.publisher.publish(
            OccupancyGrid(
                header=Header(frame_id=self.config.grid_frame_id),
                info=MapMetaData(resolution=msg.info.resolution, width=width, height=height, origin=origin),
                data=weighted_grid.astype(np.int8).reshape(-1).tolist(),
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
