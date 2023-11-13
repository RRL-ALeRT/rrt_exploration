// Copyright 2021 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*****************************************************************************************************************
 *
 * 	 Authors: Hassan Umari, Pablo Inigo Blasco
 *
 ******************************************************************************************************************/


// #include <rclcpp/rclcpp.hpp>
// #include <vector>
// #include <cmath>
// #include <nav_msgs/msg/occupancy_grid.hpp>
// #include <geometry_msgs/msg/point.hpp>
#include <functions.h>

// Rdm class, for gentaring random flot numbers
Rdm::Rdm() { int i = time(0); }
float Rdm::Randomize() {
  int i = i + 1;
  srand(i);
  return float(rand()) / float(RAND_MAX);
}

// Norm function
float Norm(std::vector<float> x1, std::vector<float> x2) {
  return pow((pow((x2[0] - x1[0]), 2) + pow((x2[1] - x1[1]), 2)), 0.5);
}

// sign function
float sign(float n) {
  if (n < 0.0) {
    return -1.0;
  } else {
    return 1.0;
  }
}

// Nearest function
std::vector<float> Nearest(std::vector<std::vector<float>> V,
                           std::vector<float> x) {
  float min = Norm(V[0], x);
  int min_index;
  float temp;

  for (int j = 0; j < V.size(); j++) {
    temp = Norm(V[j], x);
    if (temp <= min) {
      min = temp;
      min_index = j;
    }
  }

  return V[min_index];
}

// Steer function
std::vector<float> Steer(std::vector<float> x_nearest,
                         std::vector<float> x_rand, float eta) {
  std::vector<float> x_new;

  if (Norm(x_nearest, x_rand) <= eta) {
    x_new = x_rand;
  } else {
    float m = (x_rand[1] - x_nearest[1]) / (x_rand[0] - x_nearest[0]);

    x_new.push_back((sign(x_rand[0] - x_nearest[0])) *
                        (sqrt((pow(eta, 2)) / ((pow(m, 2)) + 1))) +
                    x_nearest[0]);
    x_new.push_back(m * (x_new[0] - x_nearest[0]) + x_nearest[1]);

    if (x_rand[0] == x_nearest[0]) {
      x_new[0] = x_nearest[0];
      x_new[1] = x_nearest[1] + eta;
    }
  }
  return x_new;
}

// gridValue function
int gridValue(nav_msgs::msg::OccupancyGrid &mapData, std::vector<float> Xp) {
  float resolution = mapData.info.resolution;
  float Xstartx = mapData.info.origin.position.x;
  float Xstarty = mapData.info.origin.position.y;

  float width = mapData.info.width;
  std::vector<signed char> Data = mapData.data;

  // returns grid value at "Xp" location
  // map data:  100 occupied      -1 unknown       0 free
  float indx = (floor((Xp[1] - Xstarty) / resolution) * width) +
               (floor((Xp[0] - Xstartx) / resolution));
  int out;
  out = Data[int(indx)];
  return out;
}

// ObstacleFree function-------------------------------------

int ObstacleFree(std::vector<float> xnear, std::vector<float> &xnew,
                  nav_msgs::msg::OccupancyGrid mapsub) {
  float rez = float(mapsub.info.resolution) * .2;
  float stepz = int(ceil(Norm(xnew, xnear)) / rez);
  std::vector<float> xi = xnear;
  char obs = 0;
  char unk = 0;

  geometry_msgs::msg::Point p;
  for (int c = 0; c < stepz; c++) {
    xi = Steer(xi, xnew, rez);
    auto d = gridValue(mapsub, xi);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "grid value: %d, %f, %f", d, xi[0], xi[1]);
    if (d == 100) {
      obs = 1;
    }

    if (d == -1) {
      unk = 1;
      break;
    }
  }
  int out = 0;
  xnew = xi;
  if (unk == 1) {
    out = -1;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "-unknown");
  }

  if (obs == 1) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "-obstacle");
    out = 0;
  }

  if (obs != 1 && unk != 1) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "-free");
    out = 1;
  }
  if(obs == 1 && unk == 1) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "-both");
    // out = 1;
  }
  return out;
}
