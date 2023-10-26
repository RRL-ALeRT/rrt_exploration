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

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <vector>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

// Clase Rdm para generar números flotantes aleatorios
class Rdm {
  int i;
  public:
  Rdm();
  float Randomize();
};

// Prototipo de la función Norm
float Norm(std::vector<float>, std::vector<float>);

// Prototipo de la función Sign
float sign(float);

// Prototipo de la función Nearest
std::vector<float> Nearest(std::vector<std::vector<float>>, std::vector<float>);

// Prototipo de la función Steer
std::vector<float> Steer(std::vector<float>, std::vector<float>, float);

// Prototipo de la función GridValue
int gridValue(nav_msgs::msg::OccupancyGrid&, std::vector<float>);

// Prototipo de la función ObstacleFree
char ObstacleFree(std::vector<float>, std::vector<float>&, nav_msgs::msg::OccupancyGrid);

#endif
