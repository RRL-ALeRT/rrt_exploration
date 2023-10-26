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
#!/usr/bin/env python3

# --------Include modules---------------
from copy import copy
import rclpy
from visualization_msgs.msg import Marker
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.exceptions import ROSInterruptException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import logging
import time
import tf2_ros
import numpy as np
from functions import gridValue, informationGain
from sklearn.cluster import MeanShift
from rrt_exploration.msg import PointArray

# #!/usr/bin/env python

# # --------Include modules---------------
# from copy import copy
# import rospy
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point
# from nav_msgs.msg import OccupancyGrid
# from geometry_msgs.msg import PointStamped
# import tf
# from numpy import array, vstack, delete
# from functions import gridValue, informationGain
# from sklearn.cluster import MeanShift
# from rrt_exploration.msg import PointArray

# create_subscriptions' callbacks------------------------------
mapData = OccupancyGrid()
frontiers = []
globalmaps = []
filter_node = None
global_frame = None
tfBuff = None


def goalPointsCallback(data):
    global frontiers

    if filter is not None:
        filter_node.get_logger().info(f"points callback {data}")

    if tfBuff is None or global_frame is None:
        Exception("unassigned arguments")

    transformedPoint = tfBuff.transform(data, global_frame[1:])
    x = [np.array([transformedPoint.point.x, transformedPoint.point.y])]
    if len(frontiers) > 0:
        frontiers = np.vstack((frontiers, x))
    else:
        frontiers = x


def mapCallBack(data):
    global mapData
    mapData = data


def globalMap(data):
    global global1, globalmaps, litraIndx, namespace_init_count, n_robots
    global1 = data
    if n_robots > 1:
        indx = int(data._connection_header["topic"][litraIndx]) - namespace_init_count
    elif n_robots == 1:
        indx = 0
    globalmaps[indx] = data


# Node----------------------------------------------


def main():
    global frontiers, mapData, global1, global2, global3, globalmaps, litraIndx, n_robots, namespace_init_count, filter_node, global_frame

    global tfBuff
    tfBuff = tf2_ros.Buffer()

    rclpy.init()
    # filter_node=rclpy.create_node('filter', anonymous=False)
    filter_node = rclpy.create_node("filter")

    filter_node.declare_parameter("map_topic", "/map")
    filter_node.declare_parameter("costmap_clearing_threshold", 70)
    # this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
    filter_node.declare_parameter("info_radius", 1.0)
    filter_node.declare_parameter("goals_topic", "/detected_points")
    filter_node.declare_parameter("n_robots", 1)
    filter_node.declare_parameter("namespace", "")
    filter_node.declare_parameter("namespace_init_count", 0)
    filter_node.declare_parameter("rate", 100)
    filter_node.declare_parameter("global_costmap_topic", "/global_costmap/costmap")
    filter_node.declare_parameter("robot_frame", "base_link")

    # fetching all parameters
    map_topic = str(filter_node.get_parameter("map_topic").value)
    threshold = int(filter_node.get_parameter("costmap_clearing_threshold").value)
    # this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
    info_radius = float(filter_node.get_parameter("info_radius").value)
    goals_topic = str(filter_node.get_parameter("goals_topic").value)
    n_robots = int(filter_node.get_parameter("n_robots").value)
    namespace = str(filter_node.get_parameter("namespace").value)
    namespace_init_count = int(filter_node.get_parameter("namespace_init_count").value)
    rateHz = int(filter_node.get_parameter("rate").value)
    global_costmap_topic = str(filter_node.get_parameter("global_costmap_topic").value)
    robot_frame = str(filter_node.get_parameter("robot_frame").value)

    filter_node.get_logger().info(f"info_radius: {info_radius}")
    filter_node.get_logger().info(f"threshold: {threshold}")

    # threshold = filter_node.get_parameter('~costmap_clearing_threshold', 70)
    # # this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
    # info_radius = filter_node.get_parameter('~info_radius', 1.0)
    # goals_topic = filter_node.get_parameter('~goals_topic', '/detected_points')
    # n_robots = filter_node.get_parameter('~n_robots', 1)
    # namespace = filter_node.get_parameter('~namespace', '')
    # namespace_init_count = filter_node.get_parameter('namespace_init_count', 1)
    # rateHz = filter_node.get_parameter('~rate', 100)
    # global_costmap_topic = filter_node.get_parameter(
    #     '~global_costmap_topic', '/move_base_node/global_costmap/costmap')
    # robot_frame = filter_node.get_parameter('~robot_frame', 'base_link')

    litraIndx = len(namespace)
    rate = filter_node.create_rate(rateHz)
    # -------------------------------------------
    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    map_save = filter_node.create_subscription(OccupancyGrid, map_topic, mapCallBack, qos_profile)
    # ---------------------------------------------------------------------------------------------------------------

    tfLisn = tf2_ros.TransformListener(tfBuff, filter_node)
    global_map_subscriptions = []

    for i in range(0, n_robots):
        globalmaps.append(OccupancyGrid())

    if len(namespace) > 0:
        for i in range(0, n_robots):
            global_map_subscriptions.append(
                filter_node.create_subscription(
                    OccupancyGrid,
                    namespace + str(i + namespace_init_count) + global_costmap_topic,
                    globalMap,
                    qos_profile,
                )
            )
            filter_node.get_logger().info(
                "global costmap topic: "
                + namespace
                + str(i + namespace_init_count)
                + global_costmap_topic
            )
    elif len(namespace) == 0:
        global_map_subscriptions.append(
            filter_node.create_subscription(
                OccupancyGrid, global_costmap_topic, globalMap, qos_profile
            )
        )
        filter_node.get_logger().info("global costmap topic: " + global_costmap_topic)

    #  wait if map is not received yet
    while len(mapData.data) < 1:
        filter_node.get_logger().info("Len ->Waiting for the map", throttle_duration_sec=2.0)
        time.sleep(0.1)
        rclpy.spin_once(filter_node)

    # wait if any of robots' global costmap map is not received yet
    for i in range(0, n_robots):
        while len(globalmaps[i].data) < 1:
            filter_node.get_logger().info(
                "NR ->Waiting for the global costmap", throttle_duration_sec=2.0
            )
            time.sleep(0.1)
            rclpy.spin_once(filter_node)

    global_frame = "/" + mapData.header.frame_id

    if len(namespace) > 0:
        for i in range(0, n_robots):
            tfBuff.can_transform(
                global_frame[1:],
                namespace + str(i + namespace_init_count) + "/" + robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=10.0),
            )
            tfBuff.lookup_transform(
                global_frame[1:],
                namespace + str(i + namespace_init_count) + "/" + robot_frame,
                rclpy.time.Time(),
            )

    elif len(namespace) == 0:
        bulean = tfBuff.can_transform(
            "map", "map", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=10.0)
        )
        # global_frame[1:], robot_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=10.0))
        if bulean is True:
            filter_node.get_logger().info("Can transform")
        else:
            filter_node.get_logger().info("Can not transform")

        tfBuff.lookup_transform("map", "map", rclpy.time.Time())
        # global_frame[1:], '/'+robot_frame, rclpy.time.Time())

    # filter_node.create_subscription(PointStamped, goals_topic, callback=callBack,
    #                  callback_args=[tfLisn, global_frame[1:]])
    pointssub = filter_node.create_subscription(PointStamped, goals_topic, goalPointsCallback, 1)

    qos_profile_pub = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    frontiers_pub = filter_node.create_publisher(Marker, "frontiers", qos_profile_pub)
    centroids_pub = filter_node.create_publisher(Marker, "centroids", qos_profile_pub)

    qos_profile_best_effort = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    filterpub = filter_node.create_publisher(
        PointArray, "filtered_points", qos_profile_best_effort
    )

    filter_node.get_logger().info("the map and global costmaps are received")

    filter_node.get_logger().info("waiting from frontiers")
    filter_node.get_logger().info(f"Rate: {rateHz}")

    # wait if no frontier is received yet
    while len(frontiers) < 1:
        rclpy.spin_once(filter_node)
        filter_node.get_logger().info("waiting frontiers", throttle_duration_sec=2.0)
        time.sleep(0.1)

    filter_node.get_logger().info("frontiers received")

    points = Marker()
    points_clust = Marker()
    # Set the frame ID and timestamp.  See the TF tutorials for information on these.
    points.header.frame_id = mapData.header.frame_id
    points.header.stamp = filter_node.get_clock().now().to_msg()

    points.ns = "markers2"
    points.id = 0

    points.type = Marker.POINTS

    # Set the marker action for latched frontiers.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points.action = Marker.ADD

    points.pose.orientation.w = 1.0

    points.scale.x = 0.2
    points.scale.y = 0.2

    points.color.r = 255.0 / 255.0
    points.color.g = 255.0 / 255.0
    points.color.b = 0.0 / 255.0

    points.color.a = 1.0
    points.lifetime = rclpy.duration.Duration(seconds=1).to_msg()

    p = Point()

    p.z = 0.0

    pp = []
    pl = []

    points_clust.header.frame_id = mapData.header.frame_id
    points_clust.header.stamp = filter_node.get_clock().now().to_msg()

    points_clust.ns = "markers3"
    points_clust.id = 4

    points_clust.type = Marker.POINTS

    # Set the marker action for centroids.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points_clust.action = Marker.ADD

    points_clust.pose.orientation.w = 1.0

    points_clust.scale.x = 0.2
    points_clust.scale.y = 0.2
    points_clust.color.r = 0.0 / 255.0
    points_clust.color.g = 255.0 / 255.0
    points_clust.color.b = 0.0 / 255.0

    points_clust.color.a = 1.0
    points_clust.lifetime = rclpy.duration.Duration(seconds=1).to_msg()

    temppoint = PointStamped()
    temppoint.header.frame_id = mapData.header.frame_id
    temppoint.header.stamp = filter_node.get_clock().now().to_msg()
    temppoint.point.z = 0.0

    arraypoints = PointArray()
    tempPoint = Point()
    tempPoint.z = 0.0
    # -------------------------------------------------------------------------
    # ---------------------     Main   Loop     -------------------------------
    # -------------------------------------------------------------------------

    filter_node.get_logger().info("filter main loop")
    while rclpy.ok():
        # -------------------------------------------------------------------------
        # Clustering frontier points
        centroids = []
        front = copy(frontiers)
        if len(front) > 1:
            ms = MeanShift(bandwidth=0.3)
            ms.fit(front)
            centroids = ms.cluster_centers_  # centroids array is the centers of each cluster

        # if there is only one frontier no need for clustering, i.e. centroids=frontiers
        if len(front) == 1:
            centroids = front
        frontiers = copy(centroids)

        filter_node.get_logger().info(f"frontiers: {len(frontiers)}")
        filter_node.get_logger().info(f"centroids: {len(centroids)}")
        # -------------------------------------------------------------------------
        # clearing old frontiers

        filter_node.get_logger().info(f"filtering frontiers")

        z = 0
        while z < len(centroids):
            cond = False
            temppoint.point.x = centroids[z][0]
            temppoint.point.y = centroids[z][1]

            for i in range(0, n_robots):
                transformedPoint = tfBuff.transform(temppoint, globalmaps[i].header.frame_id)
                x = np.array([transformedPoint.point.x, transformedPoint.point.y])
                gridvalue = gridValue(globalmaps[i], x)
                cond = (gridvalue > threshold or gridValue == 0) or cond

            igain = informationGain(mapData, [centroids[z][0], centroids[z][1]], info_radius * 0.5)
            filter_node.get_logger().info(f"cond: {cond} igain: {igain}")
            if cond or igain < 0.2:  # minmum information gain
                filter_node.get_logger().info(f"deleting centroid, cond: {cond} igain: {igain}")
                centroids = np.delete(centroids, (z), axis=0)
                z = z - 1
                filter_node.get_logger().info(f"filtered centroids: {len(centroids)}")

            z += 1

        # -------------------------------------------------------------------------
        # publishing
        arraypoints.points = []
        for i in centroids:
            tempPoint.x = i[0]
            tempPoint.y = i[1]
            arraypoints.points.append(copy(tempPoint))

        filter_node.get_logger().info(f"array points: {len(arraypoints.points)}")
        filterpub.publish(arraypoints)
        pp = []
        for q in range(0, len(frontiers)):
            p.x = frontiers[q][0]
            p.y = frontiers[q][1]
            pp.append(copy(p))
        points.points = pp
        pp = []
        for q in range(0, len(centroids)):
            p.x = centroids[q][0]
            p.y = centroids[q][1]
            pp.append(copy(p))
        points_clust.points = pp
        points.header.stamp = filter_node.get_clock().now().to_msg()
        points_clust.header.stamp = points.header.stamp
        frontiers_pub.publish(points)
        centroids_pub.publish(points_clust)
        time.sleep(0.1)
        rclpy.spin_once(filter_node)


# -------------------------------------------------------------------------
# rclpy.spin(filter_node)


if __name__ == "__main__":
    try:
        main()
        # rclpy.spin(node)
    except ROSInterruptException:
        pass
