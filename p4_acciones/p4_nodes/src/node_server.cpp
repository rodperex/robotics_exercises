#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "p4_interfaces/action/move.hpp"

using namespace std::chrono_literals;
using Move = p4_interfaces::action::Move;
using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

class MoveServer : public rclcpp::Node {
public:
  MoveServer()
  : Node("move_action_server")
  {
    this->action_server_ = rclcpp_action::create_server<Move>(
      this,
      "move",
      std::bind(&MoveServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveServer::handle_accepted, this, std::placeholders::_1));
  }
  ~MoveServer() {}

private:
  rclcpp_action::Server<Move>::SharedPtr action_server_;
  std::shared_ptr<GoalHandleMove> current_goal_;
  rclcpp::TimerBase::SharedPtr timer_;

  double total_;
  double completed_;
  double step_;
  rclcpp::Time start_time_;
  std::shared_ptr<Move::Feedback> feedback_;
  std::shared_ptr<Move::Result> result_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Move::Goal> goal)
  {
    if ((goal->distance != 0.0 && goal->angle != 0.0) ||
      (goal->distance == 0.0 && goal->angle == 0.0))
    {
      RCLCPP_WARN(this->get_logger(), "Just one of distance or angle must be non-zero.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Received goal with distance: %.2f, angle: %.2f",
      goal->distance, goal->angle);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle)
  {

    if (current_goal_ && timer_) {
      timer_->cancel();
      RCLCPP_WARN(this->get_logger(), "Aborting previous goal.");
    }

    current_goal_ = goal_handle;
    auto goal = goal_handle->get_goal();

    total_ = (goal->distance != 0.0) ? goal->distance : goal->angle;
    completed_ = 0.0;
    step_ = total_ / 20.0;  // 20 steps
    start_time_ = now();
    feedback_ = std::make_shared<Move::Feedback>();
    result_ = std::make_shared<Move::Result>();

    timer_ = this->create_wall_timer(
      100ms, std::bind(&MoveServer::execute_step, this));
  }

  // This is simulated execution of the action.
  // In a real robot or simulator, this would involve moving the robot.
  // Here we just simulate the action by incrementing the completed distance/angle.
  void execute_step()
  {
    if (!current_goal_ || !rclcpp::ok()) {return;}

    completed_ += std::abs(step_);
    feedback_->completed = completed_;
    feedback_->remaining = std::abs(total_) - completed_;
    feedback_->elapsed_time = (now() - start_time_).seconds();
    current_goal_->publish_feedback(feedback_);

    if (completed_ >= std::abs(total_)) {
      result_->success = true;
      current_goal_->succeed(result_);
      RCLCPP_INFO(this->get_logger(), "Goal completed.");
      timer_->cancel();
      current_goal_.reset();
    }
  }

};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
