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

# ________________________________________________________________________________


def index_of_point(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y

    i = int((Xp[1] - Xstarty) / resolution)
    j = int((Xp[0] - Xstartx) / resolution)

    return i, j


# def point_of_index(mapData, i):
#     y = mapData.info.origin.position.y + \
#         (i/mapData.info.width)*mapData.info.resolution
#     x = mapData.info.origin.position.x + \
#         (i-(i/mapData.info.width)*(mapData.info.width))*mapData.info.resolution
#     return np.array([x, y])
# ________________________________________________________________________________
def point_of_index(mapData, i, j):
    x = mapData.info.origin.position.x + j * mapData.info.resolution
    y = mapData.info.origin.position.y + i * mapData.info.resolution
    return np.array([x, y])


# def informationGain(mapData, point, r):
#     print("Information Gain: "+ str(point))

#     infoGain = 0
#     index = index_of_point(mapData, point)
#     r_region = int(r/mapData.info.resolution)
#     init_index = index-r_region*(mapData.info.width+1)

#     print (" - init_index: ", init_index)
#     print (" - r_region: ", r_region)
#     for n in range(0, 2*r_region+1):
#         start = n*mapData.info.width+init_index
#         end = start+2*r_region
#         limit = ((start/mapData.info.width)+2)*mapData.info.width
#         for i in range(start, end+1):
#             if i >= 0 and i < limit and i < len(mapData.data):
#                 if mapData.data[i] == -1:
#                     print("-1 point: ", point)
#                     if norm(np.array(point)-point_of_index(mapData, i)) <= r:
#                         infoGain += 1

#     return infoGain*(mapData.info.resolution**2)
# ________________________________________________________________________________
import numpy as np


def informationGain(mapData, point, r):
    # print("Information Gain: " + str(point))

    infoGain = 0
    i, j = index_of_point(mapData, point)
    r_region = int(r / mapData.info.resolution)

    # print(" - i: ", i)
    # print(" - j: ", j)
    # print(" - r: ", r  )
    # print(" - r_region: ", r_region)
    # print(" - mapData.info.resolution: ", mapData.info.resolution)

    # Convert mapData.data to a NumPy array without copying the data and reshape it
    data_array = np.frombuffer(mapData.data, dtype=np.int8).reshape(
        mapData.info.height, mapData.info.width
    )

    for di in range(-r_region, r_region + 1):
        for dj in range(-r_region, r_region + 1):
            new_i, new_j = i + di, j + dj
            if 0 <= new_i < mapData.info.height and 0 <= new_j < mapData.info.width:
                # print("i,j: ", new_i, new_j)
                # print("distance: "+ str(distance)+ " r: "+ str(r))
                value = data_array[new_i, new_j]

                if value == -1:
                    distance = norm(np.array(point) - point_of_index(mapData, new_i, new_j))

                    # if distance <= r:
                    # print("infoGain: ", infoGain)
                    infoGain += 1
                    # else:
                    #     # print("i: {}, j:  {}".format(new_i, new_j))
                    #     # print("ignored distance: "+ str(distance)+ " r: "+ str(r))
                    # pass

    # print("infoGain: ", infoGain)
    return infoGain * (mapData.info.resolution**2)


# def discount(mapData, assigned_pt, centroids, infoGain, r):
#     index = index_of_point(mapData, assigned_pt)
#     r_region = int(r/mapData.info.resolution)
#     init_index = index-r_region*(mapData.info.width+1)
#     for n in range(0, 2*r_region+1):
#         start = n*mapData.info.width+init_index
#         end = start+2*r_region
#         limit = ((start/mapData.info.width)+2)*mapData.info.width
#         for i in range(start, end+1):
#             if (i >= 0 and i < limit and i < len(mapData.data)):
#                 for j in range(0, len(centroids)):
#                     current_pt = centroids[j]
#                     if(mapData.data[i] == -1 and norm(point_of_index(mapData, i)-current_pt) <= r and norm(point_of_index(mapData, i)-assigned_pt) <= r):
#                         # this should be modified, subtract the area of a cell, not 1
#                         infoGain[j] -= 1
#     return infoGain

import numpy as np


def discount(mapData, assigned_pt, centroids, infoGain, r):
    r_region = int(r / mapData.info.resolution)
    data_array = np.frombuffer(mapData.data, dtype=np.int8).reshape(
        mapData.info.height, mapData.info.width
    )

    assigned_i, assigned_j = index_of_point(mapData, assigned_pt)

    for di in range(-r_region, r_region + 1):
        for dj in range(-r_region, r_region + 1):
            new_i, new_j = assigned_i + di, assigned_j + dj

            if 0 <= new_i < mapData.info.height and 0 <= new_j < mapData.info.width:
                kernel_value = data_array[new_i, new_j]
                if kernel_value == -1:
                    for k in range(len(centroids)):
                        current_pt = centroids[k]
                        if (
                            norm(point_of_index(mapData, new_i, new_j) - current_pt) <= r
                            and norm(point_of_index(mapData, new_i, new_j) - assigned_pt) <= r
                        ):
                            # Modify the infoGain value by subtracting the area of a cell, not 1
                            infoGain[k] -= 1

    return infoGain


def pathCost(path):
    if len(path) > 0:
        i = len(path) / 2
        p1 = np.array([path[i - 1].pose.position.x, path[i - 1].pose.position.y])
        p2 = np.array([path[i].pose.position.x, path[i].pose.position.y])
        return norm(p1 - p2) * (len(path) - 1)
    else:
        return float("inf")


# ________________________________________________________________________________


# def unvalid(mapData, pt):
#     index = index_of_point(mapData, pt)
#     r_region = 5
#     init_index = index-r_region*(mapData.info.width+1)
#     for n in range(0, 2*r_region+1):
#         start = n*mapData.info.width+init_index
#         end = start+2*r_region
#         limit = ((start/mapData.info.width)+2)*mapData.info.width
#         for i in range(start, end+1):
#             if (i >= 0 and i < limit and i < len(mapData.data)):
#                 if(mapData.data[i] == 1):
#                     return True
#     return False


def unvalid(mapData, pt):
    r_region = 5
    data_array = np.frombuffer(mapData.data, dtype=np.int8).reshape(
        mapData.info.height, mapData.info.width
    )

    pt_i, pt_j = index_of_point(mapData, pt)

    for di in range(-r_region, r_region + 1):
        for dj in range(-r_region, r_region + 1):
            new_i, new_j = pt_i + di, pt_j + dj

            if 0 <= new_i < mapData.info.height and 0 <= new_j < mapData.info.width:
                kernel_value = data_array[new_i, new_j]
                if kernel_value == 1:
                    return True
    return False


# ________________________________________________________________________________


def Nearest(V, x):
    n = float("inf")
    i = 0
    for i in range(0, V.shape[0]):
        n1 = norm(V[i, :] - x)
        if n1 < n:
            n = n1
            result = i
    return result


# ________________________________________________________________________________


def Nearest2(V, x):
    n = float("inf")
    result = 0
    for i in range(0, len(V)):
        n1 = norm(V[i] - x)

        if n1 < n:
            n = n1
    return i


# ________________________________________________________________________________


def gridValue(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y

    width = mapData.info.width
    Data = mapData.data

    i_index = (Xp[1] - Xstarty) / resolution
    j_index = (Xp[0] - Xstartx) / resolution

    pix_index = i_index * width + j_index
    # returns grid value at "Xp" location
    # map data:  100 occupied      -1 unknown       0 free
    index = (floor((Xp[1] - Xstarty) / resolution) * width) + (
        floor((Xp[0] - Xstartx) / resolution)
    )

    if int(index) < len(Data):
        return Data[int(index)]
    else:
        return 100
