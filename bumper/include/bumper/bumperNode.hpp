#ifndef BUMPER_BUMPERDNODE_
#define BUMPER_BUMPERDNODE_

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "kobuki_ros_interfaces/msg/bumper_event.hpp"

namespace bumper
{

class bumperNode : public rclcpp::Node
{
public:
  bumperNode();

private:
  void control_cycle();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr bumper_sub_;
  bool bump_pressed_ = false;

  void bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg);

  float init_time_;
};

}  //  namespace bumper

#endif  // BUMPER_BUMPERDNODE_