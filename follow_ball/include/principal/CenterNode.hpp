#ifndef PRINCIPAL_CENTER_NODE_HPP_
#define PRINCIPAL_CENTER_NODE_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"

namespace center
{

using namespace std::chrono_literals;  

class CenterNode : public rclcpp::Node
{
public:
  CenterNode();

private:

  enum State { BUSCAR, SEGUIR };
  int state_;

  void go_state(int new_state);
  void vector_atractivo_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void vector_repulsivo_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void control_cycle();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr vector_atractivo_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr vector_repulsivo_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Vector3 vector_resultante_;
  geometry_msgs::msg::Vector3 vector_atractivo_;
  geometry_msgs::msg::Vector3 vector_repulsivo_;  
  geometry_msgs::msg::Twist out_vel_;
};

}  

#endif  