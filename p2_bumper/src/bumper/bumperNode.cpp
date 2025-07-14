#include "geometry_msgs/msg/transform_stamped.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

#include "kobuki_ros_interfaces/msg/bumper_event.hpp"
#include "bumper/bumperNode.hpp"


namespace bumper
{
  

using namespace std::chrono_literals;

bumperNode::bumperNode()
: Node("bumper")
{
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = create_wall_timer(
    50ms, std::bind(&bumperNode::control_cycle, this));
  bumper_sub_ = create_subscription<kobuki_ros_interfaces::msg::BumperEvent>(
    "/events/bumper", 10, std::bind(&bumperNode::bumper_callback, this, std::placeholders::_1));
    
  init_time_ = this->now().seconds();  // Get the current ROS time
}

void
bumperNode::bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg)
{
  // Save the bumper state message
  bump_pressed_ = (msg->state == kobuki_ros_interfaces::msg::BumperEvent::PRESSED);
}
  
void
bumperNode::control_cycle()
{

  // Log the current bumper state and time information
  RCLCPP_INFO(
    this->get_logger(),
    "Bumper state: %s, Time elapsed: %f seconds",
    bump_pressed_ ? "PRESSED" : "RELEASED",
    this->now().seconds() - init_time_);

  geometry_msgs::msg::Twist twist;
  if(bump_pressed_ || this->now().seconds() - init_time_ > 5.0) {
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
  } else{
    twist.linear.x = 0.1;
    twist.angular.z = 0.0;
  }
  
  vel_publisher_->publish(twist);

}

}  //  namespace bumper
