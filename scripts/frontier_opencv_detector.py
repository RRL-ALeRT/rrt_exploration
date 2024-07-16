#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, Pose, Twist
from getfrontier import getfrontier
import numpy as np
import tf2_ros
from frontier_utils import *
from copy import deepcopy

EXPANSION_SIZE = 3
ROBOT_RADIUS = 0.3
SPEED = 0.3
LOOKAHEAD_DISTANCE = 0.4
TARGET_ERROR = 0.2
TARGET_ALLOWED_TIME = 10
MAP_TRIES = 10
FREE_SPACE_RADIUS = 8
UNEXPLORED_EDGES_SIZE = 6


class OpenCVFrontierDetector(Node):
    def __init__(self):
        super().__init__('detector')
        self.create_subscription(OccupancyGrid, '/projected_map_1m', self.mapCallBack, 1)
        self.targetspub = self.create_publisher(MarkerArray, "/detected_markers", 1)

        self.inflated_map_pub = self.create_publisher(OccupancyGrid, "/projected_map_1m_inflated", 1)
        self.path_publisher = self.create_publisher(Marker, 'path_marker', 1)
        
        # Initialize the transform buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(0.5, self.frontier_timer_callback)
        self.create_timer(0.1, self.path_follower_timer_callback)

        self.in_motion = False
        self.pursuit_index = 0

        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 1)

    def path_follower_timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform("map", "body", rclpy.time.Time())

            # Extract the robot's position and orientation in the "map" frame
            self.x = transform.transform.translation.x
            self.y = transform.transform.translation.y
            self.robot_yaw = yaw_from_quaternion(transform.transform.rotation.x,
                                                transform.transform.rotation.y,
                                                transform.transform.rotation.z,
                                                transform.transform.rotation.w)

            self.robot_position = [self.x, self.y]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            # Log a warning if the transform cannot be obtained
            self.get_logger().warn("Could not get transform from map to body: {}".format(ex))
            return

        if not self.in_motion:
            return
        
        if is_path_through_walls(self.inflated_map, self.current_path_map):
            print("Path passes through walls, cancelling target...")
            self.in_motion = False
            return

        linear_velocity, angular_velocity, self.pursuit_index = pure_pursuit(
            self.x,
            self.y,
            self.robot_yaw,
            self.current_path_world,
            self.pursuit_index,
            SPEED,
            LOOKAHEAD_DISTANCE
        )

        if(abs(self.x - self.current_path_world[-1][0]) < TARGET_ERROR and abs(self.y - self.current_path_world[-1][1]) < TARGET_ERROR):
            self.in_motion = False
            print("Target reached")
            linear_velocity = 0
            angular_velocity = 0
        
        if self.get_clock().now().to_msg().sec > self.target_allowed_time:
            self.in_motion = False
            print(f"Target not reached in {TARGET_ALLOWED_TIME} seconds, cancelling target...")
            linear_velocity = 0
            angular_velocity = 0

        # Publish the twist commands
        twist_command = Twist()
        twist_command.linear.x = float(linear_velocity)
        twist_command.angular.z = float(angular_velocity)
        self.twist_publisher.publish(twist_command)

    def frontier_timer_callback(self):
        if not hasattr(self, 'inflated_map'):
            self.get_logger().warn("No map received yet")
            return
        
        if not hasattr(self, 'x'):
            self.get_logger().warn("No robot position received yet")
            return
        
        if self.in_motion:
            return

        markers = MarkerArray()
        markers.markers = []

        current_map = self.inflated_map
        current_map = add_free_space_at_robot(current_map, self.x, self.y, FREE_SPACE_RADIUS)

        path_marker = Marker()
        path_marker.header = current_map.header
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.1  # Line width
        path_marker.color.a = 1.0  # Alpha
        path_marker.color.r = 0.0  # Red
        path_marker.color.g = 1.0  # Green
        path_marker.color.b = 0.0  # Blue

        frontiers = getfrontier(current_map)
        reachable_paths = []

        for i, frontier in enumerate(frontiers):
            adjusted_frontier, is_adjusted_frontier = get_nearest_free_space(current_map, frontier)

            reshaped_map = np.array(self.inflated_map.data).reshape(self.inflated_map.info.height, self.inflated_map.info.width)
            path, is_frontier_reachable = astar(reshaped_map, world_to_map_coords(current_map, self.x, self.y), world_to_map_coords(current_map, adjusted_frontier[0], adjusted_frontier[1]))

            marker = Marker()
            marker.header = current_map.header
            marker.ns = "markers"
            marker.id = i
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = marker.scale.y = 0.2
            if is_frontier_reachable:
                marker.color.g = 1.0
            elif is_adjusted_frontier:
                marker.color.b = 1.0
            else:
                marker.color.r = 1.0
            marker.color.a = 1.0
            marker.lifetime.sec = 2

            point = PointStamped()
            point.header = current_map.header
            point.point.x = float(adjusted_frontier[0])
            point.point.y = float(adjusted_frontier[1])
            point.point.z = 0.0

            marker.points.append(point.point)
            markers.markers.append(marker)

            if is_frontier_reachable:
                reachable_paths.append([path, pathLength(path)])

        if len(reachable_paths) > 0:
            reachable_paths.sort(key=lambda x: x[1])
            # Sort paths by length and select the shortest one
            path = reachable_paths[0][0]
            self.current_path_map = deepcopy(path)

            path = bspline_planning(path, len(path)*5)

            self.current_path_world = []
            for p in path:
                pose = Pose()
                pose.position.x, pose.position.y = map_to_world_coords(current_map, p[0], p[1])
                path_marker.points.append(pose.position)

                self.current_path_world.append([pose.position.x, pose.position.y])

            # Start following the path
            self.in_motion = True
            self.pursuit_index = 0
            self.target_allowed_time = self.get_clock().now().to_msg().sec + TARGET_ALLOWED_TIME

            self.map_tries = MAP_TRIES
        
        else:
            self.in_motion = False
            if not hasattr(self, 'map_tries'):
                self.map_tries = MAP_TRIES
            self.map_tries -= 1
            if self.map_tries == 0:
                print("No frontiers found. Exiting.")
                rclpy.shutdown()
            print(f"No reachable frontiers found. Retrying {self.map_tries} more times.")

        self.path_publisher.publish(path_marker)
        self.targetspub.publish(markers)

    def mapCallBack(self, data):
        self.mapData = add_unexplored_edges(data, UNEXPLORED_EDGES_SIZE)

        self.inflated_map = costmap(self.mapData, EXPANSION_SIZE)
        self.inflated_map_pub.publish(self.inflated_map)


def main(args=None):
    rclpy.init()

    detector = OpenCVFrontierDetector()

    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
