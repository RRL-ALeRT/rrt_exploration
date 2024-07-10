#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, Pose, Twist
from getfrontier import getfrontier
import numpy as np
import heapq
import tf2_ros
import math
import scipy.interpolate as si

EXPANSION_SIZE = 2
ROBOT_RADIUS = 0.3
SPEED = 0.5
LOOKAHEAD_DISTANCE = 0.4
TARGET_ERROR = 0.2
TARGET_ALLOWED_TIME = 10
MAP_TRIES = 3
FREE_SPACE_RADIUS = 5


def pure_pursuit(current_x, current_y, current_heading, path, index):
    closest_point = None
    v = SPEED  # Set the speed to a negative value to make the robot go in reverse
    # v = -speed  # Set the speed to a negative value to make the robot go in reverse
    for i in range(index, len(path)):
        x = path[i][0]
        y = path[i][1]
        distance = math.hypot(current_x - x, current_y - y)
        if LOOKAHEAD_DISTANCE < distance:
            closest_point = (x, y)
            index = i
            break
    if closest_point is not None:
        target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
        # target_heading = math.atan2(current_y - closest_point[1], current_x - closest_point[0])  # Reverse the atan2 arguments
        desired_steering_angle = target_heading - current_heading
    else:
        target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
        # target_heading = math.atan2(current_y - path[-1][1], current_x - path[-1][0])  # Reverse the atan2 arguments
        desired_steering_angle = target_heading - current_heading
        index = len(path) - 1
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    if desired_steering_angle > math.pi / 6 or desired_steering_angle < -math.pi / 6:
        sign = 1 if desired_steering_angle > 0 else -1
        desired_steering_angle = (sign * math.pi / 4)
        v = 0.0
    return v, desired_steering_angle, index


def pathLength(path):
    for i in range(len(path)):
        path[i] = (path[i][0],path[i][1])
        points = np.array(path)
    differences = np.diff(points, axis=0)
    distances = np.hypot(differences[:,0], differences[:,1])
    total_distance = np.sum(distances)
    return total_distance


def bspline_planning(array, sn):
    try:
        array = np.array(array)
        x = array[:, 0]
        y = array[:, 1]
        N = 2
        t = np.arange(len(x))
        x_tup = si.splrep(t, x, k=N)
        y_tup = si.splrep(t, y, k=N)

        x_list = list(x_tup)
        x_list[1] = np.pad(x_list[1], (0, 4), mode='constant')

        y_list = list(y_tup)
        y_list[1] = np.pad(y_list[1], (0, 4), mode='constant')

        ipl_t = np.linspace(0.0, len(x) - 1, sn)
        rx = si.splev(ipl_t, x_list)
        ry = si.splev(ipl_t, y_list)

        path = list(zip(rx, ry))
    except (TypeError, ValueError):
        path = array.tolist()

    return path


def world_to_map_coords(map_msg, world_x, world_y):
    origin_x = map_msg.info.origin.position.x
    origin_y = map_msg.info.origin.position.y
    resolution = map_msg.info.resolution

    map_x = int((world_x - origin_x) / resolution)
    map_y = int((world_y - origin_y) / resolution)
    return map_x, map_y


def map_to_world_coords(map_msg, map_x, map_y):
    origin_x = map_msg.info.origin.position.x
    origin_y = map_msg.info.origin.position.y
    resolution = map_msg.info.resolution

    world_x = map_x * resolution + origin_x
    world_y = map_y * resolution + origin_y
    return world_x, world_y


def yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


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


def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


def astar(array, start, goal):
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

            if neighbor[1] < 0 or neighbor[1] >= array.shape[0] or neighbor[0] < 0 or neighbor[0] >= array.shape[1]:
                continue

            if array[neighbor[1]][neighbor[0]] == 100:
                continue

            if array[neighbor[1]][neighbor[0]] == -1:
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

    return [], False


def get_nearest_free_space(map_data, frontier):
    width = map_data.info.width
    height = map_data.info.height

    map_data_array = np.array(map_data.data).reshape((height, width))

    search_radius = 2

    # Convert frontier point to map coordinates
    map_x, map_y = world_to_map_coords(map_data, frontier[0], frontier[1])

    for i in range(-search_radius, search_radius + 1):
        for j in range(-search_radius, search_radius + 1):
            x = map_x + i
            y = map_y + j

            # Check if the indices are within the bounds of the map
            if 0 <= x < width and 0 <= y < height:
                if map_data_array[y, x] == 0:  # Note the swapped indices for correct access
                    # Convert back to world coordinates
                    world_x, world_y = map_to_world_coords(map_data, x, y)
                    return [world_x, world_y], True

    # If no free space is found within the radius, return the original frontier
    return frontier, False


def add_free_space_at_robot(map_data, robot_x, robot_y, FREE_SPACE_RADIUS):
    width = map_data.info.width
    height = map_data.info.height

    map_data_array = np.array(map_data.data).reshape((height, width))

    robot_map_x, robot_map_y = world_to_map_coords(map_data, robot_x, robot_y)

    for i in range(-FREE_SPACE_RADIUS, FREE_SPACE_RADIUS + 1):
        for j in range(-FREE_SPACE_RADIUS, FREE_SPACE_RADIUS + 1):
            x = robot_map_x + i
            y = robot_map_y + j

            # Check if the indices are within the bounds of the map
            if 0 <= x < width and 0 <= y < height:
                map_data_array[y, x] = 0

    # Convert back to int8 array
    flattened_data = map_data_array.flatten().astype(np.int8)

    # Update the map_data with the expanded map
    map_data.data = flattened_data.tolist()  # Convert numpy array to list of int8
    return map_data


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
        
        linear_velocity, angular_velocity, self.pursuit_index = pure_pursuit(self.x, self.y, self.robot_yaw, self.current_path, self.pursuit_index)

        if(abs(self.x - self.current_path[-1][0]) < TARGET_ERROR and abs(self.y - self.current_path[-1][1]) < TARGET_ERROR):
            self.in_motion = False
            print("Target reached")
            linear_velocity = 0
            angular_velocity = 0
        
        if self.get_clock().now().to_msg().sec > self.target_allowed_time:
            self.in_motion = False
            print(f"Target not reached in {TARGET_ALLOWED_TIME} seconds.")
            linear_velocity = 0
            angular_velocity = 0

        # Publish the twist commands
        twist_command = Twist()
        twist_command.linear.x = float(linear_velocity)
        twist_command.angular.z = float(angular_velocity)
        self.twist_publisher.publish(twist_command)

    def frontier_timer_callback(self):
        if not hasattr(self, 'mapData'):
            return
        
        if not hasattr(self, 'x'):
            return
        
        if self.in_motion:
            return

        markers = MarkerArray()
        markers.markers = []

        current_map = self.mapData
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
            marker.lifetime.sec = 1

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
            path = bspline_planning(path, len(path)*5)

            self.current_path = []
            for p in path:
                pose = Pose()
                pose.position.x, pose.position.y = map_to_world_coords(current_map, p[0], p[1])
                path_marker.points.append(pose.position)

                self.current_path.append([pose.position.x, pose.position.y])

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
            print(f"No reachable frontiers found. Retrying {self.map_tries} more times.")
            if self.map_tries == 0:
                print("No frontiers found. Exiting.")
                rclpy.shutdown()

        self.path_publisher.publish(path_marker)
        self.targetspub.publish(markers)

    def mapCallBack(self, data):
        self.mapData = data

        self.inflated_map = costmap(self.mapData)
        self.inflated_map_pub.publish(self.inflated_map)


def main(args=None):
    rclpy.init()

    detector = OpenCVFrontierDetector()

    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
