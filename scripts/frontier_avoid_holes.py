#!/usr/bin/env python3

import rclpy
from avoid_holes_utils import OpenCVFrontierDetector
from frontier_utils import *
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from copy import deepcopy

UNEXPLORED_EDGES_SIZE = 6
MIN_Z = -0.06
MAX_Z = 0.25
FREE_SPACE_RADIUS = 0.5
EXPANSION_SIZE = 2


class AvoidHoles(OpenCVFrontierDetector):
    def __init__(self):
        super().__init__()

        self.sub_pc2 = self.create_subscription(PointCloud2, "/navigation/octomap_point_cloud_centers", self.pointcloud2_callback, 1)
        self.listen_to_pointcloud = False

        self.pub_grid = self.create_publisher(OccupancyGrid, '/two_d', 1)
        
        self.create_timer(0.5, self.timer_cb)

        self.cell_size = 0.1

    def timer_cb(self):
        self.listen_to_pointcloud = True

    def pointcloud2_callback(self, msg):
        if not self.listen_to_pointcloud:
            return
        self.listen_to_pointcloud = False

        try:
            # Convert PointCloud2 to numpy array
            points = np.array([[float(p[0]), float(p[1]), float(p[2])] for p in point_cloud2.read_points(msg, skip_nans=True)])

            if points.size == 0:
                self.get_logger().info('No points received in the point cloud')
                return

            if points.ndim != 2 or points.shape[1] != 3:
                self.get_logger().error(f'Unexpected points shape: {points.shape}')
                return

            min_x, min_y = np.min(points[:, :2], axis=0)
            max_x, max_y = np.max(points[:, :2], axis=0)
            grid_size_x = int((max_x - min_x) / self.cell_size) + 1
            grid_size_y = int((max_y - min_y) / self.cell_size) + 1

            # Initialize grid with -1 (unknown) and -inf for max_z values
            occupancy_grid = np.full((grid_size_y, grid_size_x), -1, dtype=int)
            max_z_values = np.full((grid_size_y, grid_size_x), -np.inf)
            explored_cells = np.zeros((grid_size_y, grid_size_x), dtype=bool)

            # Process each point to find the topmost point in each cell
            for point in points:
                x, y, z = point
                grid_x = int((x - min_x) / self.cell_size)
                grid_y = int((y - min_y) / self.cell_size)
                if max_z_values[grid_y, grid_x] < z:
                    max_z_values[grid_y, grid_x] = z
                    explored_cells[grid_y, grid_x] = True

            # Populate the occupancy grid based on topmost points
            for (grid_y, grid_x), z in np.ndenumerate(max_z_values):
                if z == -np.inf:
                    continue
                if z < MIN_Z or z > MAX_Z:
                    occupancy_grid[grid_y, grid_x] = 100
                else:
                    occupancy_grid[grid_y, grid_x] = 0

            self.publish_occupancy_grid(occupancy_grid, min_x, min_y, grid_size_x, grid_size_y, max_z_values)
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

    def publish_occupancy_grid(self, occupancy_grid, origin_x, origin_y, width, height, z_values):
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "map"

        grid_msg.info.resolution = self.cell_size
        grid_msg.info.width = width
        grid_msg.info.height = height
        grid_msg.info.origin.position.x = origin_x
        grid_msg.info.origin.position.y = origin_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        grid_msg.data = occupancy_grid.flatten().tolist()
        self.inflated_map = deepcopy(add_unexplored_edges(grid_msg, UNEXPLORED_EDGES_SIZE))
        self.inflated_map = costmap(self.inflated_map, EXPANSION_SIZE)
        self.inflated_map = add_free_space_at_robot(self.inflated_map, self.x, self.y, FREE_SPACE_RADIUS)
        self.z_values = z_values.flatten().tolist()

        self.pub_grid.publish(self.inflated_map)


def main(args=None):
    rclpy.init()

    detector = AvoidHoles()

    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
