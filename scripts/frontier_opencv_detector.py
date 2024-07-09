#!/usr/bin/env python3

# Copyright 2021 RobosoftAI Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
#  Authors: Hassan Umari, Pablo Inigo Blasco (ROS2)
#

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from getfrontier import getfrontier
import numpy as np
import heapq
import tf2_ros
import math

EXPANSION_SIZE = 2
ROBOT_RADIUS = 0.3
TARGET_TOLERANCE = 3


def add_unexplored_edges(map_msg):
    width = map_msg.info.width
    height = map_msg.info.height
    origin_x = map_msg.info.origin.position.x
    origin_y = map_msg.info.origin.position.y
    resolution = map_msg.info.resolution

    data = np.array(map_msg.data).reshape((height, width))

    edge_size = 10

    # Create an extended map with increased edges
    new_height = height + 2 * edge_size
    new_width = width + 2 * edge_size
    new_origin_x = origin_x - edge_size * resolution
    new_origin_y = origin_y - edge_size * resolution

    extended_map = np.ones((new_height, new_width)) * -1  # Initialize with -1 for unexplored

    # Copy the original map data into the center of the new map
    extended_map[edge_size:edge_size + height, edge_size:edge_size + width] = data

    # Prepare the modified OccupancyGrid message
    modified_msg = OccupancyGrid()
    modified_msg.header = map_msg.header
    modified_msg.info = map_msg.info

    # Update the map info to reflect the new dimensions and origin
    modified_msg.info.width = new_width
    modified_msg.info.height = new_height
    modified_msg.info.origin.position.x = new_origin_x
    modified_msg.info.origin.position.y = new_origin_y

    # Flatten the extended map and assign it to the message
    modified_msg.data = extended_map.flatten().astype(np.int8).tolist()

    return modified_msg


def euler_from_quaternion(x,y,z,w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z


def costmap(map_data: OccupancyGrid) -> OccupancyGrid:
    width = map_data.info.width
    height = map_data.info.height
    data = np.array(map_data.data).reshape((height, width))

    # Find walls (occupied cells)
    walls = np.where(data == 100)

    for i in range(-EXPANSION_SIZE,EXPANSION_SIZE+1):
        for j in range(-EXPANSION_SIZE,EXPANSION_SIZE+1):
            if i  == 0 and j == 0:
                continue
            x = walls[0]+i
            y = walls[1]+j
            x = np.clip(x,0,height-1)
            y = np.clip(y,0,width-1)
            data[x,y] = 100

    # Convert back to int8 array
    flattened_data = data.flatten().astype(np.int8)

    # Update the map_data with the expanded map
    map_data.data = flattened_data.tolist()  # Convert numpy array to list of int8
    return map_data


def astar(array, start, goal):
    def heuristic(a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    open_set = {start}

    while oheap:
        current = heapq.heappop(oheap)[1]
        
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start)
            data.reverse()
            return data, True
        
        close_set.add(current)
        
        if current in open_set:
            open_set.remove(current)
        
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j

            if neighbor[0] < 0 or neighbor[0] >= array.shape[0] or neighbor[1] < 0 or neighbor[1] >= array.shape[1]:
                continue

            if array[neighbor[0]][neighbor[1]] == 100:
                continue

            tentative_g_score = gscore[current] + heuristic(current, neighbor)

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in open_set:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
                open_set.add(neighbor)

    # If no path to goal was found, return closest path to goal
    if goal not in came_from:
        closest_node = None
        closest_dist = float('inf')
        for node in close_set:
            dist = heuristic(node, goal)
            if dist < closest_dist:
                closest_node = node
                closest_dist = dist
        if closest_node is not None:
            data = []
            while closest_node in came_from:
                data.append(closest_node)
                closest_node = came_from[closest_node]
            data.append(start)
            data.reverse()
            if closest_dist < TARGET_TOLERANCE:
                return data, True

    return [], False


class OpenCVFrontierDetector(Node):
    def __init__(self):
        super().__init__('detector')
        self.create_subscription(OccupancyGrid, '/projected_map_1m', self.mapCallBack, 1)
        self.targetspub = self.create_publisher(MarkerArray, "/detected_markers", 1)

        self.inflated_map_pub = self.create_publisher(OccupancyGrid, "/projected_map_1m_inflated", 1)
        
        # Initialize the transform buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(1, self.timer_callback)

    def get_nearest_free_space(self, map_data, frontier):
        width = map_data.info.width
        height = map_data.info.height
        resolution = map_data.info.resolution
        origin_x = map_data.info.origin.position.x
        origin_y = map_data.info.origin.position.y

        map_data_array = np.array(map_data.data).reshape((height, width))

        search_radius = 2

        # Convert frontier point to map coordinates
        map_x = int((frontier[0] - origin_x) / resolution)
        map_y = int((frontier[1] - origin_y) / resolution)

        for i in range(-search_radius, search_radius + 1):
            for j in range(-search_radius, search_radius + 1):
                x = map_x + i
                y = map_y + j

                # Check if the indices are within the bounds of the map
                if 0 <= x < width and 0 <= y < height:
                    if map_data_array[y, x] == 0:  # Note the swapped indices for correct access
                        # Convert back to world coordinates
                        world_x = x * resolution + origin_x
                        world_y = y * resolution + origin_y
                        return [world_x, world_y], True

        # If no free space is found within the radius, return the original frontier
        return frontier, False
    
    def timer_callback(self):
        try:
            # Get the transform from "map" frame to "odom" frame
            self.transform = self.tf_buffer.lookup_transform("map", "body", rclpy.time.Time())

            # Extract the robot's position and orientation in the "map" frame
            self.x = self.transform.transform.translation.x
            self.y = self.transform.transform.translation.y
            self.robot_yaw = euler_from_quaternion(self.transform.transform.rotation.x,
                                                self.transform.transform.rotation.y,
                                                self.transform.transform.rotation.z,
                                                self.transform.transform.rotation.w)

            self.robot_position = [self.x, self.y]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            # Log a warning if the transform cannot be obtained
            self.get_logger().warn("Could not get transform from map to body: {}".format(ex))
            return

        if not hasattr(self, 'mapData'):
            return

        markers = MarkerArray()
        markers.markers = []

        frontiers = getfrontier(self.mapData)
        for i, frontier in enumerate(frontiers):
            adjusted_frontier, is_adjusted_frontier = self.get_nearest_free_space(self.mapData, frontier)

            reshaped_map = np.array(self.inflated_map.data).reshape(self.inflated_map.info.height, self.inflated_map.info.width)
            path, is_frontier_reachable = astar(reshaped_map, (int(self.x), int(self.y)), (adjusted_frontier[0], adjusted_frontier[1]))

            marker = Marker()
            marker.header = self.mapData.header
            marker.ns = "markers"
            marker.id = i
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = marker.scale.y = 0.1
            # if is_adjusted_frontier:
            #     marker.color.r = 1.0
            if is_frontier_reachable:
                marker.color.g = 1.0
            else:
                marker.color.b = 1.0
            marker.color.a = 0.9
            marker.lifetime.sec = 1

            point = PointStamped()
            point.header = self.mapData.header
            point.point.x = float(adjusted_frontier[0])
            point.point.y = float(adjusted_frontier[1])
            point.point.z = 0.0

            marker.points.append(point.point)
            markers.markers.append(marker)
        
        self.targetspub.publish(markers)

    def mapCallBack(self, data):
        self.mapData = add_unexplored_edges(data)

        self.inflated_map = costmap(self.mapData)
        self.inflated_map_pub.publish(self.inflated_map)


def main(args=None):
    rclpy.init(args=args)

    detector = OpenCVFrontierDetector()

    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
