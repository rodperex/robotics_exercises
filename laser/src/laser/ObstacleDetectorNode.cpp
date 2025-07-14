// Copyright 2024 Intelligent Robotics Lab
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

#include <memory>

#include "laser/ObstacleDetectorNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <cmath>

namespace laser
{

using std::placeholders::_1;

ObstacleDetectorNode::ObstacleDetectorNode()
: Node("obstacle_detector_node")
{
  declare_parameter("min_distance", min_distance_);
  get_parameter("min_distance", min_distance_);

  RCLCPP_INFO(get_logger(), "ObstacleDetectorNode set to %f m", min_distance_);

  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS().reliable(),
    std::bind(&ObstacleDetectorNode::laser_callback, this, _1));
  obstacle_pub_ = create_publisher<std_msgs::msg::Bool>(
    "obstacle", 100);
  
  repulsive_vector_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
    "/rep_vector", 20); 
}

void
ObstacleDetectorNode::laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
{
  int min_idx = std::min_element(scan->ranges.begin(), scan->ranges.end()) - scan->ranges.begin();
  float distance_min = scan->ranges[min_idx];

  auto obstacle_msg = std_msgs::msg::Bool();

  float angle = scan->angle_min + scan->angle_increment * min_idx;
  if (distance_min > 0.5) {
    while (angle > M_PI) {angle -= 2.0 * M_PI;}
    while (angle < -M_PI) {angle += 2.0 * M_PI;}

    RCLCPP_INFO(get_logger(), "Obstacle in (%.2f, %.2f)", distance_min, angle*180.0/M_PI);

    obstacle_msg.data = true;
  } else {
    obstacle_msg.data = false;
  }

  obstacle_pub_->publish(obstacle_msg);

  // intensidad del vector repulsivo, K la constante de proporcionalidad
  float intensity = std::fmin(0.0, K / std::pow(distance_min, 2));
  
  // Publicar el vector repulsivo (angulo 0 -> Hacia atras)
  // angulo positivo (izquierda) restar 180 y negativo (derecha) sumar 180
  // poner el cero delante y ya ver x e y para calcular el vector con cos y sen

  if (angle < 0) {
    angle += M_PI;
  } else {
    angle -= M_PI;
  } 
  geometry_msgs::msg::Vector3 repulsive_vector_;
  repulsive_vector_.x = -std::cos(angle)*intensity;
  repulsive_vector_.y = -std::sin(angle)*intensity;
  repulsive_vector_.z = 0.0;
  repulsive_vector_pub_->publish(repulsive_vector_);
}

}  // namespace laser
