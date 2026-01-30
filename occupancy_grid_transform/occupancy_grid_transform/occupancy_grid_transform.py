from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData
from geometry_msgs.msg import Point, Pose, Quaternion
import numpy as np
import math
from nav_utils.nav_utils.geometry import get_yaw_radians_from_quaternion, rotate_by_yaw
from rclpy.node import Node
import rclpy
from .occupancy_grid_trasform_config import OccupancyGridTransformConfig
from typing import Optional
from std_msgs.msg import Header
from itertools import product
import nav_utils.nav_utils.config

class OccupancyGridTransform(Node):
    def __init__(self):
        super().__init__("occupancy_grid_transform")

        self.config: OccupancyGridTransformConfig = nav_utils.nav_utils.config.load(self, OccupancyGridTransformConfig)

        self._odom: Optional[Odometry] = None

        self.create_subscription(OccupancyGrid, "occupancy_grid", self.occupancy_grid_callback, 10)
        self.create_subscription(Odometry, "odom", self.odom_callback, 10)

        self.publisher = self.create_publisher(OccupancyGrid, "occupancy_grid_transform", 10)

    def odom_callback(self, msg: Odometry) -> None:
        self._odom = msg

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        grid = np.asarray(msg.data, dtype=np.int8).reshape((msg.info.width, msg.info.height), order='F')
        grid = np.flipud(grid)
        grid = self.inflate_occupancy_grid(grid)

        origin = Pose()
        half_height_m = grid.shape[0] * msg.info.resolution / 2

        if self._odom is not None:
            yaw = get_yaw_radians_from_quaternion(self._odom.pose.pose.orientation)
            local_x = self.config.robot_forward_offset_m
            local_y = -half_height_m
            rotated = rotate_by_yaw(Point(x=local_x, y=local_y), yaw)

            origin.position.x = self._odom.pose.pose.position.x + rotated.x
            origin.position.y = self._odom.pose.pose.position.y + rotated.y
            origin.position.z = 0.0
            origin.orientation = self._odom.pose.pose.orientation
        else:
            # fallback, assume robot is at origin
            origin.position.x = self.config.robot_forward_offset_m
            origin.position.y = -half_height_m
            origin.position.z = 0.0
            origin.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)

        self.publisher.publish(OccupancyGrid(
            header=Header(
                frame_id=self.config.grid_frame_id
            ),
            info=MapMetaData(
                resolution=msg.info.resolution,
                width=grid.shape[1],
                height=grid.shape[0],
                origin=origin
            ),
            data=grid.astype(np.int8).reshape(-1).tolist()
        ))
        
    def inflate_occupancy_grid(self, grid: np.ndarray) -> np.ndarray:
        r_hard = self.config.inflation_radius_cells
        r_soft = self.config.inflation_falloff_radius_cells
        decay  = self.config.inflation_decay_factor
        height, width = grid.shape 

        output = grid.astype(np.int8, copy=True)

        def inflate(x: int, y: int) -> None:
            for dx, dy in product(range(-r_soft, r_soft + 1), repeat=2):
                dist = math.hypot(dx, dy)
                if dist > r_soft:
                    continue

                nx, ny = x + dx, y + dy
                if not (0 <= nx < width) or not (0 <= ny < height):
                    continue

                value = int(100 if dist <= r_hard else round(100 * (decay ** (dist - r_hard))))
                output[ny, nx] = max(output[ny, nx], value)

        ys, xs = np.where(grid == 100)
        for x, y in zip(xs.tolist(), ys.tolist()):
            inflate(x, y)

        return output.astype(np.int8)

def main() -> None:
    rclpy.init()
    node = OccupancyGridTransform()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
