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

#ifndef FOLLOW_PERSON__PERSONFOLLOWERNODELC_HPP_
#define FOLLOW_PERSON__PERSONFOLLOWERNODELC_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/macros.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "follow_person/PIDController.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace follow_person
{

class PersonFollowerNodeLC : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(PersonFollowerNodeLC)

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  PersonFollowerNodeLC();

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state);

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state);

private:
  // TF listener
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void state_machine();
  bool check_following();
  void follow();
  void search();

  static const int FOLLOWING_ = 0;
  static const int SEARCHING_ = 1;
  int state_;

  static const int LEFT_ = 0;
  static const int RIGHT_ = 1;
  int side_;

  PIDController vlin_pid_, vrot_pid_;
  double min_lin, min_rot;
  double max_lin, max_rot;
  double limit_distance;
};

}  //  namespace follow_person

#endif  // FOLLOW_PERSON__PERSONFOLLOWERNODELC_HPP_
