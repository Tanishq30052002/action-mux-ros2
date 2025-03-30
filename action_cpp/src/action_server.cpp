#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "action_cpp/action/move_to_pose.hpp"
#include "action_cpp/visibility_control.h"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class MoveToPoseServer : public rclcpp::Node {
 public:
  using MoveToPose = action_cpp::action::MoveToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveToPose>;

  MoveToPoseServer() : Node("move_to_pose_server") {
    action_server_ = rclcpp_action::create_server<MoveToPose>(
        this, "move_to_pose",
        std::bind(&MoveToPoseServer::handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&MoveToPoseServer::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&MoveToPoseServer::handle_accepted, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "MoveToPose Action Server is running...");
  }

 private:
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
  std::shared_ptr<GoalHandle> active_goal_;
  std_msgs::msg::Bool goal_currently_active_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const MoveToPose::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received new goal!");

    if (active_goal_ && active_goal_->is_active()) {
      RCLCPP_WARN(this->get_logger(),
                  "New goal received, aborting the previous one!");
      active_goal_->abort(std::make_shared<MoveToPose::Result>());
    }

    // Only accept a new goal if `active_goal_` is reset
    if (active_goal_) {
      RCLCPP_WARN(this->get_logger(), "Ignoring duplicate goal!");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_WARN(this->get_logger(), "Goal canceled!");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    active_goal_ = goal_handle;
    std::thread(&MoveToPoseServer::execute, this, goal_handle).detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal...");

    auto feedback = std::make_shared<MoveToPose::Feedback>();
    rclcpp::Rate rate(1);  // 1Hz loop (one update per second)

    for (int i = 0; i < 5; ++i) {
      if (goal_handle->is_canceling()) {
        RCLCPP_WARN(this->get_logger(), "Goal aborted before completion.");
        goal_handle->canceled(std::make_shared<MoveToPose::Result>());
        active_goal_.reset();  // **Reset active goal after canceling**
        return;
      }

      feedback->current_pose = true;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Processing goal... %d sec elapsed",
                  i + 1);

      rate.sleep();
    }

    if (goal_handle->is_active()) {
      auto result = std::make_shared<MoveToPose::Result>();
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(),
                  "Goal processed successfully after 5 seconds!");
    }

    active_goal_.reset();  // **Ensure active goal is reset after execution**
    RCLCPP_INFO(this->get_logger(), "Waiting for the next goal...");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveToPoseServer>());
  rclcpp::shutdown();
  return 0;
}
