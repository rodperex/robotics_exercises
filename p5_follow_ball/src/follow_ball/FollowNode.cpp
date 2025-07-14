#include <utility>
#include "principal/CenterNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace center
{

using namespace std::chrono_literals;
using std::placeholders::_1;

CenterNode::CenterNode()
: Node("center_node"),
state_(SEGUIR)
//state_ (BUSCAR)
{
  vector_atractivo_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
    "/attr_vector", 10, std::bind(&CenterNode::vector_atractivo_callback, this, _1));

  vector_repulsivo_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
    "/rep_vector", 10, std::bind(&CenterNode::vector_repulsivo_callback, this, _1));
  
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = create_wall_timer(20ms, std::bind(&CenterNode::control_cycle, this));
}

void CenterNode::vector_atractivo_callback(const geometry_msgs::msg::Vector3::SharedPtr msg){
  vector_atractivo_ = *msg;
  RCLCPP_INFO(this->get_logger(), "Received attractive vector: [%f, %f, %f]", msg->x, msg->y, msg->z);
}

void CenterNode::vector_repulsivo_callback(const geometry_msgs::msg::Vector3::SharedPtr msg){
  vector_repulsivo_ = *msg;
  RCLCPP_INFO(this->get_logger(), "Received repulsive vector: [%f, %f, %f]", msg->x, msg->y, msg->z);
}

void CenterNode::control_cycle() {

  geometry_msgs::msg::Twist out_vel;

  switch (state_) {
    case BUSCAR:
      if (vector_atractivo_.x == 0.0 && vector_atractivo_.y == 0.0) {
        out_vel.linear.x = 0;
        out_vel.angular.z = 0.3f;
      } else {
        go_state (SEGUIR);
      }
      break;
    case SEGUIR:
      /*vector_resultante_ = vector_repulsivo_;
      float magnitud = std::sqrt(vector_resultante_.x * vector_resultante_.x + vector_resultante_.y * vector_resultante_.y);
      if (vector_resultante_.x < 0.0) {
        magnitud = -magnitud;
      }

      float angulo = std::atan2(vector_resultante_.y, vector_resultante_.x);

      out_vel.linear.x = std::min(magnitud*0.1f, 0.1f);
      RCLCPP_INFO(this->get_logger(), "magnitud: %f", magnitud);
      out_vel.angular.z = std::clamp(-angulo, -0.1f, 0.1f);
      RCLCPP_INFO(this->get_logger(), "angulo: %f", angulo); */

      if (vector_atractivo_.x != 0.0 || vector_atractivo_.y != 0.0) {
        vector_resultante_.x = vector_atractivo_.x + vector_repulsivo_.x;
        vector_resultante_.y = vector_atractivo_.y + vector_repulsivo_.y;

        float magnitud = std::sqrt(vector_resultante_.x * vector_resultante_.x + vector_resultante_.y * vector_resultante_.y);
        if (vector_resultante_.x < 0.0) {
          magnitud = -magnitud;
        }
        float angulo = std::atan2(vector_resultante_.y, vector_resultante_.x);
        
        out_vel.linear.x = std::min(magnitud*0.2f, 0.2f);
        //-angulo para que gire hacia el lado correcto
        out_vel.angular.z = std::clamp(-angulo, -0.3f, 0.3f); 
      } else {
        go_state (BUSCAR);
      } 

      break;

  }

  vel_pub_->publish(out_vel);
  RCLCPP_INFO(this->get_logger(), "Publishing velocity: linear.x: %f, angular.z: %f", out_vel.linear.x, out_vel.angular.z);

}
void CenterNode::go_state(int new_state){
  state_ = new_state;
  RCLCPP_INFO(this->get_logger(), "Estado cambiado a: %s", new_state == BUSCAR ? "BUSCAR" : "SEGUIR");
}

}  