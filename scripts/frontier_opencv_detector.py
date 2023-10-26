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

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from getfrontier import getfrontier
from rclpy.exceptions import ROSInterruptException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# #!/usr/bin/env python


# #--------Include modules---------------
# import rospy
# from visualization_msgs.msg import Marker
# from nav_msgs.msg import OccupancyGrid
# from geometry_msgs.msg import PointStamped
# from getfrontier import getfrontier

# -----------------------------------------------------
# create_subscriptions' callbacks------------------------------
mapData = OccupancyGrid()


def mapCallBack(data):
    global mapData
    mapData = data


# Node----------------------------------------------
def node():
    global mapData
    rclpy.init()
    exploration_goal = PointStamped()
    # detector_node=rclpy.create_node('detector', anonymous=False)
    detector_node = rclpy.create_node("detector")

    detector_node.declare_parameter("~map_topic", "/robot_1/map")

    map_topic = str(detector_node.get_parameter("~map_topic").value)

    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    detector_node.create_subscription(OccupancyGrid, map_topic, mapCallBack, qos_profile)

    targetspub = detector_node.create_publisher("/detected_points", PointStamped, queue_size=10)
    pub = detector_node.create_publisher("shapes", Marker, queue_size=10)
    # wait until map is received, when a map is received, mapData.header.seq will not be < 1
    while len(mapData.data) < 1:
        # while mapData.header.seq<1 or len(mapData.data)<1:
        pass

    rate = rclpy.Rate(50)
    points = Marker()

    # Set the frame ID and timestamp.  See the TF tutorials for information on these.
    points.header.frame_id = mapData.header.frame_id
    points.header.stamp = rclpy.time.Time.now()

    points.ns = "markers"
    points.id = 0

    points.type = Marker.POINTS
    # Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points.action = Marker.ADD

    points.pose.orientation.w = 1.0
    points.scale.x = points.scale.y = 0.3
    points.color.r = 255.0 / 255.0
    points.color.g = 0.0 / 255.0
    points.color.b = 0.0 / 255.0
    points.color.a = 1
    points.lifetime == rclpy.duration.Duration()

    # -------------------------------OpenCV frontier detection------------------------------------------
    while rclpy.ok():
        frontiers = getfrontier(mapData)
        for i in range(len(frontiers)):
            x = frontiers[i]
            exploration_goal.header.frame_id = mapData.header.frame_id
            exploration_goal.header.stamp = rclpy.time.Time(0)
            exploration_goal.point.x = x[0]
            exploration_goal.point.y = x[1]
            exploration_goal.point.z = 0

            targetspub.publish(exploration_goal)
            points.points = [exploration_goal.point]
            pub.publish(points)
        rate.sleep()

        # rate.sleep()
    rclpy.spin(detector_node)


# _____________________________________________________________________________

if __name__ == "__main__":
    try:
        node()
        # rclpy.spin(node)
    except ROSInterruptException:
        pass
