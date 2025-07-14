#include "follow_person/PersonFollowerNode.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace follow_person
{

PersonFollowerNode::PersonFollowerNode()
: rclcpp::Node("person_follower_node"),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  vlin_pid_(0.0, 1.0, 0.0, 0.7),
  vrot_pid_(0.0, 1.0, 0.3, 1.0)
{
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

  velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>("velocity", 100);

  side_ = RIGHT_;
  state_ = SEARCHING_;

  timer_ = create_wall_timer(
    100ms, std::bind(&PersonFollowerNode::state_machine, this));
}

void PersonFollowerNode::follow()
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

    if (angle > 0) { side_ = LEFT_; } else if (angle < 0) { side_ = RIGHT_; }

    double dist_limited = dist - limit_distance;
    if (fabs(dist_limited) <= 0.15) { dist_limited = 0; }

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
    RCLCPP_WARN_STREAM(get_logger(), "Error in TF base_footprint -> target [" << error << "]");
  }
}

void PersonFollowerNode::search()
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

void PersonFollowerNode::state_machine()
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
      break;
  }
}

bool PersonFollowerNode::check_following()
{
  std::string error;
  if (tf_buffer_.canTransform("odom", "target", tf2::TimePointZero, &error)) {
    auto odom2target_msg = tf_buffer_.lookupTransform("odom", "target", tf2::TimePointZero);

    if (this->now() - odom2target_msg.header.stamp > 1s) {
      return false;
    }
  } else {
    RCLCPP_WARN_STREAM(get_logger(), "Error in TF odom -> target [" << error << "]");
    return false;
  }

  return true;
}

}  // namespace follow_person
