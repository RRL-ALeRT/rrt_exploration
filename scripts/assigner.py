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

from nav_msgs.msg import OccupancyGrid
from rrt_exploration.msg import PointArray
from rclpy.exceptions import ROSInterruptException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time
import tf2_ros
from time import sleep
from numpy import array, linalg as LA, all as All, inf
from copy import copy
from functions import informationGain, discount
from robot import robot
from rclpy.callback_groups import ReentrantCallbackGroup

from numpy.linalg import norm
from rclpy.action import ActionServer, CancelResponse, GoalResponse

# import ExploreNextPoint.action
from rrt_exploration.action import ExploreNextPoint
import threading


class AssignerNode:
    def __init__(self):
        # create_subscriptions' callbacks------------------------------
        self.assigner_node = None
        self.mapData = OccupancyGrid()

        self.robots = []
        self.received_filtered_points = []

        self.visited = []
        self.revenue_record = None
        self.centroid_record = None
        self.id_record = None

        self.running_navigation_action = False
        self.navgoal = None
        self.robotid = None

        self.decision_thread = None
        self.decision_data_lock = threading.Lock()

    def receivedPointsCallback(self, data):
        # self.decision_data_lock.acquire()
        self.received_filtered_points = []
        for point in data.points:
            self.received_filtered_points.append(array([point.x, point.y]))
        # self.decision_data_lock.release()

    def mapCallBack(self, data):
        self.mapData = data

    # Node----------------------------------------------

    def spin_some_local(self):
        rclpy.spin_once(self.assigner_node)
        # for rob in self.robots:
        #     rclpy.spin_once(rob)

    def run(self):
        rclpy.init()

        # self.assigner_node=rclpy.create_node('assigner', anonymous=False)
        self.assigner_node = rclpy.create_node("assigner")

        self.assigner_node.get_logger().info("assigner node started")

        self.assigner_node.declare_parameter("map_topic", "/map")
        # this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
        self.assigner_node.declare_parameter("info_radius", 1.0)
        self.assigner_node.declare_parameter("cost_distance_factor", 1.2)
        self.assigner_node.declare_parameter("info_multiplier", 3.0)
        # at least as much as the laser scanner range
        self.assigner_node.declare_parameter("hysteresis_radius", 3.0)
        # bigger than 1 (biase robot to continue exploring current region
        self.assigner_node.declare_parameter("hysteresis_gain", 2.0)
        self.assigner_node.declare_parameter("frontiers_topic", "/filtered_points")
        self.assigner_node.declare_parameter("n_robots", 1)
        self.assigner_node.declare_parameter("namespace", "")
        self.assigner_node.declare_parameter("namespace_init_count", 1)
        self.assigner_node.declare_parameter("delay_after_assignement", 0.5)
        self.assigner_node.declare_parameter("rate", 100)

        # fetching all parameters
        map_topic = str(self.assigner_node.get_parameter("map_topic").value)

        # this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
        self.info_radius = float(self.assigner_node.get_parameter("info_radius").value)
        self.info_multiplier = float(self.assigner_node.get_parameter("info_multiplier").value)
        self.hysteresis_radius = float(
            self.assigner_node.get_parameter("hysteresis_radius").value
        )  # at least as much as the laser scanner range

        # bigger than 1 (biase robot to continue exploring current region
        self.hysteresis_gain = float(self.assigner_node.get_parameter("hysteresis_gain").value)
        self.cost_distance_factor = float(
            self.assigner_node.get_parameter("cost_distance_factor").value
        )
        frontiers_topic = str(self.assigner_node.get_parameter("frontiers_topic").value)
        self.n_robots = int(self.assigner_node.get_parameter("n_robots").value)
        self.namespace = str(self.assigner_node.get_parameter("namespace").value)
        self.namespace_init_count = int(
            self.assigner_node.get_parameter("namespace_init_count").value
        )
        self.delay_after_assignement = float(
            self.assigner_node.get_parameter("delay_after_assignement").value
        )
        rateHz = int(self.assigner_node.get_parameter("rate").value)

        #  map_topic= self.assigner_node.get_parameter('~map_topic','/map')
        # self.info_radius= self.assigner_node.get_parameter('~self.info_radius',1.0)                    #this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
        # self.info_multiplier=self.assigner_node.get_parameter('~self.info_multiplier',3.0)
        # self.hysteresis_radius=self.assigner_node.get_parameter('~self.hysteresis_radius',3.0)            #at least as much as the laser scanner range
        # self.hysteresis_gain=self.assigner_node.get_parameter('~self.hysteresis_gain',2.0)                #bigger than 1 (biase robot to continue exploring current region
        # frontiers_topic= self.assigner_node.get_parameter('~frontiers_topic','/filtered_points')
        # self.n_robots = self.assigner_node.get_parameter('~self.n_robots',1)
        # self.namespace = self.assigner_node.get_parameter('~self.namespace','')
        # self.namespace_init_count = self.assigner_node.get_parameter('~self.namespace_init_count',1)
        # self.delay_after_assignement=self.assigner_node.get_parameter('~self.delay_after_assignement',0.5)
        # rateHz = self.assigner_node.get_parameter('~rate',100)

        self.rate = self.assigner_node.create_rate(rateHz)
        # -------------------------------------------

        self.assigner_node.get_logger().info("creating subscribers and actions")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.occupancy_grid_subscriber = self.assigner_node.create_subscription(
            OccupancyGrid, map_topic, self.mapCallBack, qos_profile
        )
        self.frontiers_subscriber = self.assigner_node.create_subscription(
            PointArray, frontiers_topic, self.receivedPointsCallback, qos_profile
        )
        # ---------------------------------------------------------------------------------------------------------------

        self.wait_frontiers()

        self.wait_maps()

        self.assigner_node.get_logger().info("creating action server")

        # be careful with this because it is blocking of this main thread
        if len(self.namespace) > 0:
            for i in range(0, self.n_robots):
                self.robots.append(
                    robot(self.namespace + str(i + self.namespace_init_count), self.assigner_node)
                )
        else:
            self.robots.append(robot(self.namespace, self.assigner_node))

        self.explore_next_point_action = ActionServer(
            self.assigner_node,
            ExploreNextPoint,
            "explore_next_point",
            self.explore_next_point_execute_callback,
        )

        self.assigner_node.get_logger().info("starting main loop")

        # self.assigner_node.create_timer(1.0, self.try_prepare_next_action_decision, callback_group=ReentrantCallbackGroup())
        # self.pretake_decision_thread = threading.Thread(target=self.try_prepare_next_action_decision)
        # self.pretake_decision_thread.start()

        self.assigner_main_loop()

    def wait_maps(self):
        self.assigner_node.get_logger().info("waiting maps")

        # wait if map is not received yet
        while len(self.mapData.data) < 1:
            time.sleep(0.1)
            # rclpy.spin_once(self.assigner_node)
            # for rob in self.robots:
            #     rclpy.spin.once(rob)
            self.spin_some_local()

    def wait_frontiers(self):
        self.assigner_node.get_logger().info("waiting frontiers")
        # wait if no frontier is received yet
        while len(self.received_filtered_points) < 1:
            time.sleep(0.1)
            self.spin_some_local()
            centroids = copy(self.received_filtered_points)
            self.assigner_node.get_logger().info("waiting frontiers", throttle_duration_sec=1.0)

    def explore_next_point_execute_callback(self, goal_handle):
        self.running_navigation_action = True
        self.assigner_node.get_logger().info("Action server execute goal...")
        result = ExploreNextPoint.Result()

        self.wait_check_robots_initialized()

        if goal_handle.is_cancel_requested:
            self.assigner_node.get_logger().info("Goal canceled")
            goal_handle.canceled()
            self.assigner_node.get_logger().info("Goal canceled")
            return ExploreNextPoint.Result()

        success = self.execute_navigation(goal_handle)

        # check send goal result succeeded
        self.assigner_node.get_logger().info("Explore next point continue")
        if success:
            goal_handle.succeed()
            # result.result = "success"
            self.assigner_node.get_logger().info("Explore next point success")
        else:
            goal_handle.abort()
            # result.result = "aborted"
            self.assigner_node.get_logger().info("Explore next point aborted")

        self.running_navigation_action = False
        return result

    def explore_next_point_cancel_callback(self, goal_handle):
        self.assigner_node.get_logger().info("Received cancel request")
        self.running_navigation_action = False
        return CancelResponse.ACCEPT

    def explore_next_point_goal_callback(self, goal_handle):
        if self.running_navigation_action:
            self.assigner_node.get_logger().info("Already consuming next goal request")
            return GoalResponse.REJECT

        self.assigner_node.get_logger().info("Received new goal request")
        self.running_navigation_action = True
        return GoalResponse.ACCEPT

    def wait_check_robots_initialized(self):
        while len(self.robots) < 1:
            self.assigner_node.get_logger().info("waiting for robots")
            time.sleep(0.1)
            self.spin_some_local()

    def assigner_main_loop(self):
        self.assigner_node.get_logger().info("main loop")
        while rclpy.ok():
            if not self.running_navigation_action:
                self.assigner_node.get_logger().info(
                    "Waiting for action request", throttle_duration_sec=1.0
                )
                self.log_current_status()
                # self.execute_navigation(None)

            self.spin_some_local()
            time.sleep(0.1)

    def log_current_status(self):
        # self.decision_data_lock.acquire()
        msg = ""
        msg += "---- CURRENT STATUS ----------------\n"
        msg += " received points: " + str(self.received_filtered_points) + "\n"
        msg += " visited: " + str(self.visited) + "\n"
        msg += " centroids record: " + str(self.centroid_record) + "\n"
        msg += " id_record: " + str(self.id_record) + "\n"
        msg += " revenue_record: " + str(self.revenue_record) + "\n"
        msg += " info_radius: " + str(self.info_radius) + "\n"

        msg += " ready next nav_goal: " + str(self.navgoal) + "\n"
        msg += "---------------------------" + "\n"
        # self.decision_data_lock.release()

        self.assigner_node.get_logger().info(msg, throttle_duration_sec=2.0)

    def try_prepare_next_action_decision(self):
        if self.decision_thread is None or not self.decision_thread.is_alive():
            robot_positions = [p.getPosition() for p in self.robots]
            self.decision_thread = threading.Thread(
                target=self.try_prepare_next_action_decision_thread, args=(robot_positions)
            )
            self.decision_thread.start()

    def try_prepare_next_action_decision_thread(self, robot_positions):

        # while True:
        self.assigner_node.get_logger().info(".")
        self.decision_data_lock.acquire()
        visited = copy(self.visited)
        centroids = copy(
            [
                p
                for p in self.received_filtered_points
                if not any(All(p == v) for v in self.visited)
            ]
        )
        self.decision_data_lock.release()

        # -------------------------------------------------------------------------
        # Get information gain for each frontier point
        infoGain = self.compute_centroid_gains(centroids)

        # -------------------------------------------------------------------------
        # get number of available/busy self.robots
        available_robots = []  # available self.robots

        busy_robots = []  # busy self.robots
        for i in range(0, self.n_robots):
            if self.robots[i].getState() == 1:
                busy_robots.append(i)
            else:
                available_robots.append(i)

        self.assigner_node.get_logger().info("available robots: " + str(available_robots))
        self.assigner_node.get_logger().info("busy robots: " + str(busy_robots))
        # -------------------------------------------------------------------------

        self.assigner_node.get_logger().info("--- Discount for available robots---")
        # get dicount and update informationGain
        for i in busy_robots + available_robots:
            infoGain = discount(
                self.mapData, self.robots[i].assigned_point, centroids, infoGain, self.info_radius
            )
            # -------------------------------------------------------------------------
        self.revenue_record = []
        self.centroid_record = []
        self.id_record = []

        if any(available_robots):
            for ir in available_robots:
                self.assigner_node.get_logger().info(
                    "available robot " + str(ir) + "  state: " + str(self.robots[ir].getState())
                )
                for ip in range(0, len(centroids)):
                    current_point = centroids[ip]

                    cost = norm(robot_positions[ir] - current_point) * self.cost_distance_factor
                    self.assigner_node.get_logger().info(
                        "centroid " + str(current_point) + "  cost: " + str(cost)
                    )

                    information_gain = infoGain[ip]
                    if norm(robot_positions[ir] - current_point) <= self.hysteresis_radius:
                        information_gain *= self.hysteresis_gain

                    revenue = information_gain * self.info_multiplier - cost
                    self.assigner_node.get_logger().info(
                        "centroid " + str(current_point) + "  revenue: " + str(revenue)
                    )

                    # Añadido para comprobar puntos ya visitados
                    # self.decision_data_lock.acquire()
                    point_is_visited = any(All(p == current_point) for p in visited)
                    # self.decision_data_lock.release()
                    self.assigner_node.get_logger().info(
                        "centroid "
                        + str(current_point)
                        + "  point_is_visited: "
                        + str(point_is_visited)
                    )

                    if not point_is_visited:
                        self.centroid_record.append(centroids[ip])
                        self.revenue_record.append(revenue)
                        self.id_record.append(ir)
        else:
            # check state of busy robots
            self.assigner_node.get_logger().info("---- check state of busy robots ----")
            for ir in busy_robots:
                self.assigner_node.get_logger().info(
                    "busy robot " + str(ir) + "  state: " + str(self.robots[ir].getState())
                )
                for ip in range(0, len(centroids)):
                    current_point = centroids[ip]
                    cost = norm(robot_positions[ir] - current_point)

                    information_gain = infoGain[ip]
                    if norm(robot_positions[ir] - current_point) <= self.hysteresis_radius:
                        # recalculate information gain if the candidate point is in the hysteresis radius
                        information_gain = (
                            informationGain(
                                self.mapData,
                                [current_point[0], current_point[1]],
                                self.info_radius,
                            )
                            * self.hysteresis_gain
                        )
                        self.assigner_node.get_logger().info(
                            "recalculated centroid "
                            + str(current_point)
                            + "  cost: "
                            + str(cost)
                            + " gain: "
                            + str(information_gain)
                        )
                        # information_gain *= self.hysteresis_gain

                    # # recalculate information gain if the goal is in the hysteresis radius
                    # if ((norm(centroids[ip]-self.robots[ir].assigned_point)) < self.hysteresis_radius):
                    #     information_gain = informationGain(
                    #             self.mapData, [current_point[0], current_point[1]], self.info_radius)*self.hysteresis_gain

                    revenue = information_gain * self.info_multiplier - cost

                    self.assigner_node.get_logger().info(
                        "centroid "
                        + str(current_point)
                        + " cost: "
                        + str(cost)
                        + " gain: "
                        + str(information_gain)
                        + " revenue: "
                        + str(revenue)
                    )

                    # self.decision_data_lock.acquire()
                    point_is_visited = any(All(p == current_point) for p in visited)
                    # self.decision_data_lock.release()

                    if not point_is_visited:
                        self.centroid_record.append(centroids[ip])
                        self.revenue_record.append(revenue)
                        self.id_record.append(ir)

                        # revenue_record.append(revenue)
                        # centroid_record.append(centroids[ip])
                        # id_record.append(ir)

        self.assigner_node.get_logger().info(" ---- resume after selection ---")
        self.assigner_node.get_logger().info("revenue record: " + str(self.revenue_record))
        self.assigner_node.get_logger().info("centroid record: " + str(self.centroid_record))
        self.assigner_node.get_logger().info("robot IDs record: " + str(self.id_record))

        # -------------------------------------------------------------------------

        # self.take_decision_and_execute()
        self.assigner_node.get_logger().info("---- taking decision ----")
        if len(self.id_record) > 0:
            winner_id = self.revenue_record.index(max(self.revenue_record))

            candidates = zip(self.revenue_record, self.centroid_record, self.id_record)
            # sort centroids record by revenue
            ordered_candidate = sorted(candidates, key=lambda x: x[0], reverse=True)

            self.decision_data_lock.acquire()
            # self.navgoal = self.centroid_record[winner_id]
            self.navgoal = copy(ordered_candidate)

            # self.robotid = self.id_record[winner_id]
            self.decision_data_lock.release()

            self.assigner_node.get_logger().info(
                f"NEXT DECISION UPDATED ALREADY: {self.navgoal} {self.robotid}"
            )

            # self.take_decision_and_execute_2()
        else:
            self.assigner_node.get_logger().info("No points to explore")
            self.log_current_status()

        # self.assigner_node.get_logger().info("releasing")
        # self.decision_data_lock.release()

    def execute_navigation(self, goal_handle):
        navgoal = None
        self.assigner_node.get_logger().info("Taking decision and execute 2")
        while True:
            self.try_prepare_next_action_decision()
            self.decision_data_lock.acquire()

            try:
                if self.navgoal is None or len(self.navgoal) == 0:
                    self.decision_data_lock.release()
                    # self.assigner_node.get_logger().info("releasing lock")
                    self.assigner_node.get_logger().info(
                        "Executing navigation but navgoal was not decided yet",
                        throttle_duration_sec=1.0,
                    )
                    # self.spin_some_local()
                    continue

                target = self.navgoal.pop(0)
                navgoal = target[1]
                robotid = target[2]

                self.visited.append(navgoal)
                # self.navgoal = None
                # self.robotid = None

            except Exception as e:
                self.assigner_node.get_logger().info("Error in take_decision_and_execute_2")
                self.assigner_node.get_logger().info(e)

            self.decision_data_lock.release()
            time.sleep(0.1)

            break

        self.assigner_node.get_logger().info(
            "Starting navigation wait loop for robot " + str(robotid)
        )

        send_goal_future = self.robots[robotid].sendGoal(navgoal)

        self.assigner_node.get_logger().info("goal future: " + str(send_goal_future))
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.assigner_node.get_logger().error("Goal was rejected! breaking attempt")
            return False

        result_future = goal_handle.get_result_async()

        finished = False  # Añadido para espera
        while not finished:
            try:
                self.try_prepare_next_action_decision()
                # finished = self.robots[robotid].lookGoal()
                finished = result_future.done()
            except Exception as e:
                self.assigner_node.get_logger().info("Error in take_decision_and_execute_2")
                self.assigner_node.get_logger().info(e)
            self.assigner_node.get_logger().info("Navigating... finished: " + str(finished))
            self.log_current_status()
            # self.spin_some_local()
            time.sleep(0.4)

        self.assigner_node.get_logger().info("Navigation FINISHED!")
        return finished

    def compute_centroid_gains(self, centroids):
        self.assigner_node.get_logger().info("*computifng centroids gains*")
        infoGain = []
        for ip in range(0, len(centroids)):
            infoGain.append(
                informationGain(
                    self.mapData, [centroids[ip][0], centroids[ip][1]], self.info_radius
                )
            )
            self.assigner_node.get_logger().info(
                f"centroid {ip} ({centroids[ip][0]},{centroids[ip][1]}) -> {infoGain[ip]}"
            )

        self.assigner_node.get_logger().info("infoGain: " + str(infoGain))

        return infoGain


if __name__ == "__main__":
    try:
        assigner_node = AssignerNode()
        assigner_node.run()
    except ROSInterruptException as ex:
        print(f"ROSInterruptException: {ex}")
        pass
