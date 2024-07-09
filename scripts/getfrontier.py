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

from copy import copy

import numpy as np
import cv2



def getfrontier(mapData):
    data = mapData.data
    w = mapData.info.width
    h = mapData.info.height
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y

    img = np.zeros((h, w, 1), np.uint8)

    for i in range(0, h):
        for j in range(0, w):
            if data[i * w + j] == 100:
                img[i, j] = 0
            elif data[i * w + j] == 0:
                img[i, j] = 255
            elif data[i * w + j] == -1:
                img[i, j] = 205

    o = cv2.inRange(img, 0, 1)
    edges = cv2.Canny(img, 0, 255)
    contours, _ = cv2.findContours(o, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(o, contours, -1, (255, 255, 255), 5)
    o = cv2.bitwise_not(o)
    res = cv2.bitwise_and(o, edges)
    # ------------------------------

    frontier = copy(res)
    contours, _ = cv2.findContours(frontier, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frontier, contours, -1, (255, 255, 255), 2)

    contours, _ = cv2.findContours(frontier, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    all_pts = []
    if len(contours) > 0:
        i = 0

        for i in range(0, len(contours)):
            cnt = contours[i]
            M = cv2.moments(cnt)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            xr = cx * resolution + Xstartx
            yr = cy * resolution + Xstarty
            pt = [np.array([xr, yr])]
            if len(all_pts) > 0:
                all_pts = np.vstack([all_pts, pt])
            else:

                all_pts = pt

    return all_pts
