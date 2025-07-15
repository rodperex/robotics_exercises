#include "rclcpp/rclcpp.hpp"

#include "bumper/bumperNode.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<bumper::bumperNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
