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

#include <cmath>
#include <functions.h>
#include <iostream>
#include <mtrand.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include <opencv2/opencv.hpp>

// global variables
nav_msgs::msg::OccupancyGrid mapData;
geometry_msgs::msg::PointStamped clickedpoint;
geometry_msgs::msg::PointStamped exploration_goal;
visualization_msgs::msg::Marker points, line;
float xdim, ydim, resolution, Xstartx, Xstarty, init_map_x, init_map_y;
using namespace std::chrono_literals;
int inflation_window_size;

Rdm r; // for genrating random numbers

// create_subscriptions callback
// functions---------------------------------------
void mapCallBack(const nav_msgs::msg::OccupancyGrid::ConstPtr &msg) {

  cv::Mat mapImage(msg->info.height,msg->info.width, CV_8UC1, const_cast<int8_t*>(msg->data.data()));
  cv::Mat inflatedmapImage;

  cv::dilate(mapImage, inflatedmapImage, cv::Mat(), cv::Point(-1,-1), inflation_window_size);

  inflatedmapImage.copyTo(mapImage);
  
  mapData = *msg;
}

void rvizCallBack(const geometry_msgs::msg::PointStamped::ConstPtr &msg) {
  geometry_msgs::msg::Point p;
  p.x = msg->point.x;
  p.y = msg->point.y;
  p.z = msg->point.z;

  points.points.push_back(p);
}

int main(int argc, char **argv) {
  unsigned long init[4] = {0x123, 0x234, 0x345, 0x456}, length = 7;
  MTRand_int32 irand(init,
                     length); // 32-bit int generator
                              // this is an example of initializing by an array
                              // you may use MTRand(seed) with any 32bit integer
                              // as a seed for a simpler initialization
  MTRand drand;               // double in [0, 1) generator, already init

  // generate the same numbers as in the original C test program
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("global_rrt_frontier_detector");

  nh->declare_parameter("inflation_window_size", inflation_window_size);  
  nh->get_parameter("inflation_window_size", inflation_window_size);

  // fetching all parameters
  float eta, init_map_x, init_map_y, range;
  std::string map_topic, base_frame_topic;

  std::string ns = nh->get_name();
  // std::string ns = nh->get_namespace();

  // ros::param::param<float>(ns+"/eta", eta, 0.5);
  // ros::param::param<std::string>(ns+"/map_topic", map_topic, "/robot_1/map");

  RCLCPP_INFO(nh->get_logger(), "Initalizating parameters...");
  RCLCPP_INFO(nh->get_logger(), "Inflation window size: %d", inflation_window_size);

  nh->declare_parameter<float>(ns + "/eta", 0.5);
  nh->get_parameter<float>(ns + "/eta", eta);

  nh->declare_parameter<std::string>(ns + "/map_topic", "/map");
  nh->get_parameter<std::string>(ns + "/map_topic", map_topic);

  nh->declare_parameter<int>(ns + "/inflation_window_size", 25);
  nh->get_parameter<int>(ns + "/inflation_window_size", inflation_window_size);

  RCLCPP_INFO(nh->get_logger(), "Parameters initialized");
  RCLCPP_INFO_STREAM(nh->get_logger(), "eta: " << eta);
  // RCLCPP_INFO_STREAM(nh->get_logger(), "inflation_window_size: " << inflation_window_size);
  //---------------------------------------------------------------
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub =
      nh->create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic, 100,
                                                            mapCallBack);
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rviz_sub =
      nh->create_subscription<geometry_msgs::msg::PointStamped>(
          "/clicked_point", 100, rvizCallBack);

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr targetspub =
      nh->create_publisher<geometry_msgs::msg::PointStamped>("/detected_points",
                                                             10);
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub =
      nh->create_publisher<visualization_msgs::msg::Marker>(ns + "_shapes", 10);

  rclcpp::Rate rate(100);

  // wait until map is received, when a map is received, mapData.header.seq will
  // not be < 1

  // while (mapData.header.seq<1 or mapData.data.size()<1)  {  ros::spinOnce();
  // rclcpp::Duration(0.1).sleep();} //  # original line

  // # start previous line substitution

  rclcpp::QoS qos(10);
  qos.reliable().transient_local();

  auto map_subscription = nh->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", qos, mapCallBack);

  RCLCPP_INFO(nh->get_logger(), "Waiting map");
  rclcpp::executors::SingleThreadedExecutor executor;
  // while (!executor.should_exit()) { // Averiguar sustituto
  while (mapData.data.size() < 1) {
    // while (true) {
    // ros::spinOnce();
    rclcpp::spin_some(nh);
    executor.spin_some(
        std::chrono::milliseconds(100)); // Procesa eventos durante 0.1 segundos
    rclcpp::sleep_for(100ms);
  }

  RCLCPP_INFO(nh->get_logger(), "Map acquired");
  // # end previous line substitution

  // visualizations  points and lines..
  points.header.frame_id = mapData.header.frame_id;
  line.header.frame_id = mapData.header.frame_id;
  points.header.stamp = rclcpp::Time(0);
  line.header.stamp = rclcpp::Time(0);

  points.ns = line.ns = "markers";
  points.id = 0;
  line.id = 1;

  points.type = points.POINTS;
  line.type = line.LINE_LIST;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3
  // (DELETEALL)
  points.action = points.ADD;
  line.action = line.ADD;
  points.pose.orientation.w = 1.0;
  line.pose.orientation.w = 1.0;
  line.scale.x = 0.03;
  line.scale.y = 0.03;
  points.scale.x = 0.3;
  points.scale.y = 0.3;

  line.color.r = 9.0 / 255.0;
  line.color.g = 91.0 / 255.0;
  line.color.b = 236.0 / 255.0;
  points.color.r = 255.0 / 255.0;
  points.color.g = 0.0 / 255.0;
  points.color.b = 0.0 / 255.0;
  points.color.a = 1.0;
  line.color.a = 1.0;
  points.lifetime = rclcpp::Duration(0, 0);
  line.lifetime = rclcpp::Duration(0, 0);

  geometry_msgs::msg::Point p;

  RCLCPP_INFO(nh->get_logger(), "Initial points");

  while (points.points.size() < 5) {
    // ros::spinOnce();
    rclcpp::spin_some(nh);
    RCLCPP_INFO_THROTTLE(nh->get_logger(), *(nh->get_clock()),  1000,
                         "waiting for more points: %ld/5",
                         points.points.size());

    pub->publish(points);
    rclcpp::sleep_for(100ms);
  }

  RCLCPP_INFO(nh->get_logger(), "Initial points received");

  std::vector<float> temp1;
  temp1.push_back(points.points[0].x);
  temp1.push_back(points.points[0].y);

  std::vector<float> temp2;
  temp2.push_back(points.points[2].x);
  temp2.push_back(points.points[0].y);

  init_map_x = Norm(temp1, temp2);
  temp1.clear();
  temp2.clear();

  temp1.push_back(points.points[0].x);
  temp1.push_back(points.points[0].y);

  temp2.push_back(points.points[0].x);
  temp2.push_back(points.points[2].y);

  init_map_y = Norm(temp1, temp2);
  temp1.clear();
  temp2.clear();

  Xstartx = (points.points[0].x + points.points[2].x) * .5;
  Xstarty = (points.points[0].y + points.points[2].y) * .5;

  geometry_msgs::msg::Point trans;
  trans = points.points[4];
  std::vector<std::vector<float>> V;
  std::vector<float> xnew;
  xnew.push_back(trans.x);
  xnew.push_back(trans.y);
  V.push_back(xnew);

  points.points.clear();
  pub->publish(points);

  float xr, yr;
  std::vector<float> x_rand, x_nearest, x_new;

  RCLCPP_INFO(nh->get_logger(), "Main loop");

  // Main loop
  while (rclcpp::ok()) {
    RCLCPP_INFO_STREAM_THROTTLE(nh->get_logger(), *(nh->get_clock()), 2000,".");
    rclcpp::spin_some(nh);

    // Sample free
    x_rand.clear();
    xr = (drand() * init_map_x) - (init_map_x * 0.5) + Xstartx;
    yr = (drand() * init_map_y) - (init_map_y * 0.5) + Xstarty;

    x_rand.push_back(xr);
    x_rand.push_back(yr);

    // Nearest
    x_nearest = Nearest(V, x_rand);

    // RCLCPP_INFO_THROTTLE_STREAM(nh->get_logger(),
    RCLCPP_INFO_STREAM_THROTTLE(nh->get_logger(), *(nh->get_clock()), 2000, "Nearest " << x_nearest[0] << ", " << x_nearest[1]);


    // Steer

    x_new = Steer(x_nearest, x_rand, eta);

    RCLCPP_INFO_STREAM_THROTTLE(nh->get_logger(), 
    *(nh->get_clock()) , 2000 ,  "New"  << x_new[0] << ", " << x_new[1]);

    // ObstacleFree    1:free     -1:unkown (frontier region)      0:obstacle
    int checking = ObstacleFree(x_nearest, x_new, mapData);

    RCLCPP_INFO(nh->get_logger(), "*checking: %d", checking);

    if (checking == -1) {
      RCLCPP_INFO(nh->get_logger(),  "checking point unknown");
      exploration_goal.header.stamp = nh->now();
      exploration_goal.header.frame_id = mapData.header.frame_id;
      exploration_goal.point.x = x_new[0];
      exploration_goal.point.y = x_new[1];
      exploration_goal.point.z = 0.0;
      p.x = x_new[0];
      p.y = x_new[1];
      p.z = 0.0;
      points.points.push_back(p);
      pub->publish(points);
      targetspub->publish(exploration_goal);
      points.points.clear();
    }

    else if (checking == 1) {
      RCLCPP_INFO_STREAM_THROTTLE(nh->get_logger(), *(nh->get_clock()) ,  2000 , "checking free");
      V.push_back(x_new);

      p.x = x_new[0];
      p.y = x_new[1];
      p.z = 0.0;
      line.points.push_back(p);
      p.x = x_nearest[0];
      p.y = x_nearest[1];
      p.z = 0.0;
      line.points.push_back(p);
    }

    pub->publish(line);

    // ros::spinOnce();
    rclcpp::spin_some(nh);
    rate.sleep();
  }
  return 0;
}
