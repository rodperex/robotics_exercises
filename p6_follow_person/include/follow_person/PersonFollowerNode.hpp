#ifndef FOLLOW_PERSON__PERSONFOLLOWERNODE_HPP_
#define FOLLOW_PERSON__PERSONFOLLOWERNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "follow_person/PIDController.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace follow_person
{

class PersonFollowerNode : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(PersonFollowerNode)

  PersonFollowerNode();

private:
  // TF listener
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
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

#endif  // FOLLOW_PERSON__PERSONFOLLOWERNODE_HPP_
