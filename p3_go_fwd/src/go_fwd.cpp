#include "rclcpp/rclcpp.hpp"

#include "go_fwd/goFwdNode.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<go_fwd::goFwdNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
