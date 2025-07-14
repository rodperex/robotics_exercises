#ifndef GO_FWD_GOFWDNODE_
#define GO_FWD_GOFWDNODE_

#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace go_fwd
{

class goFwdNode : public rclcpp::Node
{
public:
  goFwdNode();

private:
  void control_cycle();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  int state_;
};

}  //  namespace go_fwd

#endif  // GO_FWD_GOFWDNODE_