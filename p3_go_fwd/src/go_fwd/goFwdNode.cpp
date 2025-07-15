#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

#include "go_fwd/goFwdNode.hpp"


namespace go_fwd
{

using namespace std::chrono_literals;

goFwdNode::goFwdNode()
: Node("go_fwd"),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = create_wall_timer(
    50ms, std::bind(&goFwdNode::control_cycle, this));
    
  state_ = 0;  // Initialize state variable
  RCLCPP_INFO(get_logger(), "Fwd");
}

void
goFwdNode::control_cycle()
{
  tf2::Stamped<tf2::Transform> bf2target;
  std::string error;

  if (tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &error)) {
    auto bf2target_msg = tf_buffer_.lookupTransform(
      "odom", "base_footprint", tf2::TimePointZero);

    tf2::fromMsg(bf2target_msg, bf2target);

    double x = bf2target.getOrigin().x();

    double angle = bf2target.getRotation().getAngle();
    RCLCPP_INFO(get_logger(), "X: %f, Angle: %f", x, angle);

    geometry_msgs::msg::Twist twist;
    if(state_ == 0) {
      twist.linear.x = 0.3;
      twist.angular.z = 0.0;
    } else if (state_ == 1) {
      twist.linear.x = 0.0;
      twist.angular.z = 0.3;
    } else if (state_ == 2) {
      // In state 2 (or any other state), stop the robot
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;
    } else {
      rclcpp::shutdown();
    }
    
    vel_publisher_->publish(twist);

    if(state_ == 0 && x > 5.0) {
      RCLCPP_INFO(get_logger(), "Turning");
      state_ = 1;  // Transition to state 1
    } else if(state_ == 1 && angle > 3.14) {
      RCLCPP_INFO(get_logger(), "Stopping");
      state_ = 2;  // Transition to state 2
    } else if(state_ == 2) {
      state_ = 3;  // Transition to state 3
    }

  } else {
    RCLCPP_WARN_STREAM(get_logger(), "Error in TF odom -> base_footprint [<< " << error << "]");
  }
}

}  //  namespace go_fwd