#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "p4_interfaces/action/move.hpp"

using namespace std::chrono_literals;
using Move = p4_interfaces::action::Move;

class MoveClient : public rclcpp::Node {
public:
  MoveClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("move_client", options)
  {
    client_ = rclcpp_action::create_client<Move>(this, "move");
  }

  ~MoveClient() {}

  void send_goal(double distance, double angle)
  {
    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available.");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = Move::Goal();
    goal_msg.distance = distance;
    goal_msg.angle = angle;

    RCLCPP_INFO(this->get_logger(), "Sending goal, distance = %.2f, angle = %.2f", distance, angle);

    auto send_goal_options = rclcpp_action::Client<Move>::SendGoalOptions();
    send_goal_options.feedback_callback =
      std::bind(&MoveClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&MoveClient::result_callback, this, std::placeholders::_1);

    auto future_goal_handle = client_->async_send_goal(goal_msg, send_goal_options);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Error sending goal.");
      rclcpp::shutdown();
      return;
    }

    auto goal_handle = future_goal_handle.get();
    if (!goal_handle) {
      RCLCPP_WARN(this->get_logger(), "Server rejected goal.");
      rclcpp::shutdown();
      return;
    }
  }

private:
  rclcpp_action::Client<Move>::SharedPtr client_;

  void feedback_callback(
    rclcpp_action::ClientGoalHandle<Move>::SharedPtr,
    const std::shared_ptr<const Move::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(),
        "Feedback -> completed: %.2f, remaining: %.2f, elapsed_time: %.2f",
        feedback->completed,
        feedback->remaining,
        feedback->elapsed_time);
  }

  void result_callback(const rclcpp_action::ClientGoalHandle<Move>::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Action succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Action aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Action canceled");
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "Other result code");
        break;
    }

    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3) {
    std::cerr << "Use: move_client <distance> <angle>" << std::endl;
    return 1;
  }

  double distance = std::stod(argv[1]);
  double angle = std::stod(argv[2]);

  auto node = std::make_shared<MoveClient>();
  node->send_goal(distance, angle);
  rclcpp::spin(node);
  return 0;
}
