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
from nav2_simple_commander.robot_navigator import BasicNavigator
import tf2_ros
from tf2_ros import TransformListener

# from rclpy.action import Action # ??
from geometry_msgs.msg import PoseStamped

# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan

# import action_msgs.msg as action_msgs
import logging
import time
import numpy as np
from numpy.linalg import (
    norm,
)  # aqui se importaba inf pero he sustituido por float('inf') cada aparición de inf en el código
from math import floor
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

# import rospy
# import tf
# from numpy import array
# import actionlib
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from nav_msgs.srv import GetPlan
# from geometry_msgs.msg import PoseStamped
# from numpy import floor
# from numpy.linalg import norm
# from numpy import float('inf')
# ________________________________________________________________________________


class robot:
    # goal = MoveBaseGoal()

    # nav = BasicNavigator()

    def __init__(self, name, node):
        self.node = node
        self.assigned_point = []
        self.name = name
        self.start = PoseStamped()
        self.end = PoseStamped()

        self.node.declare_parameter(name + "global_frame", "map")
        self.node.declare_parameter(name + "robot_frame", "base_link")
        # self.declare_parameter('~rate', 100)

        self.global_frame = self.node.get_parameter(name + "global_frame").value
        self.robot_frame = self.node.get_parameter(name + "robot_frame").value
        # self.plan_service = self.get_parameter('~plan_service')
        # self.global_frame = self.get_parameter('~global_frame', '/map')
        # self.robot_frame = self.get_parameter('~robot_frame', 'base_link')
        # self.plan_service = self.get_parameter('~plan_service', '/move_base_node/NavfnROS/make_plan')
        # self.nav = BasicNavigator()
        self.buffer = tf2_ros.Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self.buffer.can_transform(
            self.global_frame,
            self.robot_frame,
            rclpy.time.Time(),
            rclpy.duration.Duration(seconds=10.0),
        )

        self.nav_to_pose_client = ActionClient(self.node, NavigateToPose, "navigate_to_pose")

        cond = 0
        while cond == 0:
            time.sleep(0.1)
            rclpy.spin_once(self.node)
            try:
                self.node.get_logger().info("Fun ->Waiting for the robot transform")

                transf = self.buffer.lookup_transform(
                    self.global_frame, self.robot_frame, rclpy.time.Time()
                )

                self.node.get_logger().info(f"Transf {transf}")
                cond = 1
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                cond == 0

        self.node.get_logger().info("Fun ->Transform found")

        self.robot_pose = PoseStamped()
        self.robot_pose.pose.position.x = transf.transform.translation.x
        self.robot_pose.pose.position.y = transf.transform.translation.y
        self.robot_pose.pose.orientation.x = transf.transform.rotation.x
        self.robot_pose.pose.orientation.y = transf.transform.rotation.y
        self.robot_pose.pose.orientation.z = transf.transform.rotation.z
        self.robot_pose.pose.orientation.w = transf.transform.rotation.x

        # np.array([transf.transform.translation.x, transf.transform.translation.y])
        self.assigned_point = [self.robot_pose.pose.position.x, self.robot_pose.pose.position.y]
        # self.client = self.create_client(self.name+'/move_base', MoveBaseAction)
        # self.client.wait_for_service()
        # self.nav.setInitialPose(self.position)
        self.node.get_logger().info("Waiting btnavigator active")
        # self.nav.waitUntilNav2Active(localizer="/bt_navigator")
        self.goal = PoseStamped()
        self.goal.header.frame_id = self.global_frame
        self.goal.header.stamp = self.node.get_clock().now().to_msg()

        self.node.get_logger().info("Navigator active")

        # self.client.wait_for_service(self.name+self.plan_service)
        # self.make_plan = self.create_client(GetPlan,
        #     self.name+self.plan_service)
        self.start.header.frame_id = self.global_frame
        self.end.header.frame_id = self.global_frame

    def getPosition(self):
        cond = 0
        while cond == 0:
            try:
                rclpy.spin_once(self.node)
                transf = self.buffer.lookup_transform(
                    self.global_frame, self.robot_frame, rclpy.time.Time()
                )
                cond = 1
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                cond == 0
        self.robot_pose = np.array(
            [transf.transform.translation.x, transf.transform.translation.y]
        )

        return self.robot_pose

    def sendGoal(self, point):
        self.goal.pose.position.x = point[0]
        self.goal.pose.position.y = point[1]
        self.goal.pose.orientation.w = 1.0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goal

        # self.client.send_goal(self.goal)
        # self.nav.goToPose(self.goal)
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future)

        self.assigned_point = [self.goal.pose.position.x, self.goal.pose.position.y]

        return send_goal_future

    def cancelGoal(self):
        # self.client.cancel_goal()
        self.nav.cancelTask()
        p = self.getPosition()
        self.assigned_point = [p.x, p.y]

    def getState(self):

        # return self.nav.getState()
        return 1

    def makePlan(self, start, end):
        self.start.pose.position.x = start[0]
        self.start.pose.position.y = start[1]
        self.end.pose.position.x = end[0]
        self.end.pose.position.y = end[1]
        # start = self.listener.transformPose(self.name+'/map', robot.start)
        # end = self.listener.transformPose(self.name+'/map', robot.end)
        # plan = self.make_plan(start=start, goal=end, tolerance=0.0)
        path = self.nav.getPath(self.start, self.end)
        smoothed_path = self.nav.smoothPath(path)
        # return plan.plan.poses
        return path

    def waitGoal(self):
        while not self.nav.isTaskComplete():
            time.sleep(0.1)
            rclpy.spin_once(self.node)
        return 1

    def lookGoal(self):
        if self.nav.isTaskComplete():
            return 1
        else:
            return 0

    def isSuccessResult(self):
        return self.nav.isTaskComplete()
        # return self.nav.isTaskSuccessful()
