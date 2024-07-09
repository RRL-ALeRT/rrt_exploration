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

class OpenCVFrontierDetector(Node):
    def __init__(self):
        super().__init__('detector')
        self.create_subscription(OccupancyGrid, '/projected_map_1m', self.mapCallBack, 1)
        self.targetspub = self.create_publisher(MarkerArray, "/detected_markers", 1)
        
        self.create_timer(1, self.timer_callback)
    
    def timer_callback(self):
        if not hasattr(self, 'mapData'):
            return

        markers = MarkerArray()
        markers.markers = []

        frontiers = getfrontier(self.mapData)
        for i, frontier in enumerate(frontiers):
            marker = Marker()
            marker.header = self.mapData.header
            marker.ns = "markers"
            marker.id = i
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = 0.3
            marker.color.r = 255.0 / 255.0
            marker.color.g = 0.0 / 255.0
            marker.color.b = 0.0 / 255.0
            marker.color.a = 1.0
            marker.lifetime.sec = 1

            point = PointStamped()
            point.header = self.mapData.header
            point.point.x = frontier[0]
            point.point.y = frontier[1]
            point.point.z = 0.0
            
            marker.points.append(point.point)
            markers.markers.append(marker)
        
        self.targetspub.publish(markers)

    def mapCallBack(self, data):
        self.mapData = data

def main(args=None):
    rclpy.init(args=args)

    detector = OpenCVFrontierDetector()

    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
