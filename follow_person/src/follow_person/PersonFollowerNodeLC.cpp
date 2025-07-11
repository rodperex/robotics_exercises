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

#include "follow_person/PersonFollowerNodeLC.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace follow_person
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

PersonFollowerNodeLC::PersonFollowerNodeLC()
: LifecycleNode("person_follower_node"),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  vlin_pid_(0.0, 1.0, 0.0, 0.7),
  vrot_pid_(0.0, 1.0, 0.3, 1.0)
{
}

void
PersonFollowerNodeLC::follow()
{
  tf2::Stamped<tf2::Transform> bf2target;
  geometry_msgs::msg::Twist twist;
  std::string error;


  if (tf_buffer_.canTransform("base_footprint", "target", tf2::TimePointZero, &error)) {

    auto bf2target_msg = tf_buffer_.lookupTransform(
      "base_footprint", "target", tf2::TimePointZero);

    tf2::fromMsg(bf2target_msg, bf2target);
    double x = bf2target.getOrigin().x();
    double y = bf2target.getOrigin().y();

    double dist = sqrt(x * x + y * y);
    double angle = atan2(y, x);

   

    if (angle > 0) {side_ = LEFT_;} else if (angle < 0) {side_ = RIGHT_;}

    double dist_limited = dist - limit_distance;
    if (fabs(dist_limited) <= 0.15) {dist_limited = 0;}

    double vel_lin = std::clamp(vrot_pid_.get_output(dist_limited), min_lin, max_lin);
    double vel_rot = std::clamp(vlin_pid_.get_output(angle) * 1.5, min_rot, max_rot);
    
    RCLCPP_INFO_STREAM(
      get_logger(),
      "target at " << std::fixed << std::setprecision(2) << dist << " m; angle = " << angle
      << " rad; vel_lin = " << vel_lin << " m/s; vel_rot = " << vel_rot << " rad/s");

    twist.linear.x = vel_lin;
    twist.angular.z = vel_rot;
    velocity_pub_->publish(twist);

  } else {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Error in TF base_footprint -> target [<< " << error << "]");
  }
}


void
PersonFollowerNodeLC::search()
{
  RCLCPP_INFO(get_logger(), "Searching...");

  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0;

  if (side_ == LEFT_) {
    twist.angular.z = min_rot;
  } else {
    twist.angular.z = max_rot;
  }

  velocity_pub_->publish(twist);
}

void
PersonFollowerNodeLC::state_machine()
{
  switch (state_) {
    case FOLLOWING_:
      follow();

      if (!check_following()) {
        state_ = SEARCHING_;
      }
      break;

    case SEARCHING_:
      search();

      if (check_following()) {
        state_ = FOLLOWING_;
      }
  }
}


bool
PersonFollowerNodeLC::check_following()
{
  std::string error;
  if (tf_buffer_.canTransform("odom", "target", tf2::TimePointZero, &error)) {

    auto odom2target_msg = tf_buffer_.lookupTransform(
      "odom", "target", tf2::TimePointZero);

    if (now() - odom2target_msg.header.stamp > 1s) {
      return false;
    }

  } else {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Error in TF odom -> target [<< " << error << "]");
    return false;
  }

  return true;
}

CallbackReturn
PersonFollowerNodeLC::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Configuring...");

  declare_parameter("min_lin", -0.3);
  declare_parameter("max_lin", 0.3);
  declare_parameter("min_rot", -0.8);
  declare_parameter("max_rot", 0.8);
  declare_parameter("limit_distance", 1.0);

  get_parameter("min_lin", min_lin);
  get_parameter("max_lin", max_lin);
  get_parameter("min_rot", min_rot);
  get_parameter("max_rot", max_rot);
  get_parameter("limit_distance", limit_distance);

  RCLCPP_INFO(
    get_logger(),
    "min lin = %f, max lin = %f, min rot = %f, max rot = %f",
    min_lin, max_lin, min_rot, max_rot);

  velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "velocity", 100);


  side_ = RIGHT_;
  state_ = SEARCHING_;

  return CallbackReturn::SUCCESS;
}

CallbackReturn
PersonFollowerNodeLC::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Activating...");

  timer_ = create_wall_timer(
    100ms, std::bind(&PersonFollowerNodeLC::state_machine, this));

  velocity_pub_->on_activate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
PersonFollowerNodeLC::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Deactivating...");

  timer_ = nullptr;
  velocity_pub_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
PersonFollowerNodeLC::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Cleaning Up...");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
PersonFollowerNodeLC::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Shutting Down...");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
PersonFollowerNodeLC::on_error(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Error State");

  return CallbackReturn::SUCCESS;
}

}  //  namespace follow_person
