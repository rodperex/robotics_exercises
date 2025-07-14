#include <memory>

#include "principal/CenterNode.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto center_node = std::make_shared<center::CenterNode>();
  rclcpp::spin(center_node);

  rclcpp::shutdown();

  return 0;
}
