#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "action_cpp/action/move_to_pose.hpp"

using namespace std::chrono_literals;

class MoveToPoseServer : public rclcpp::Node {
 public:
  using MoveToPose = action_cpp::action::MoveToPose;
  using MoveToPoseGoalHandle = rclcpp_action::ServerGoalHandle<MoveToPose>;

  MoveToPoseServer() : Node("move_to_pose_server") {
    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<MoveToPose>(
        this, "move_to_pose",
        std::bind(&MoveToPoseServer::handle_goal, this, _1, _2),
        std::bind(&MoveToPoseServer::handle_cancel, this, _1),
        std::bind(&MoveToPoseServer::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "MoveToPose Action Server is running...");
  }

 private:
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
  std::shared_ptr<MoveToPoseGoalHandle> active_goal_;
  std::mutex goal_mutex_;

  // Handle new goals
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const MoveToPose::Goal> goal) {
    (void)uuid;
    std::lock_guard<std::mutex> lock(goal_mutex_);

    // If an active goal exists, cancel it properly
    if (active_goal_ && active_goal_->is_active()) {
      RCLCPP_WARN(this->get_logger(),
                  "[handle_goal] Requesting cancellation of previous goal.");

      // Request cancellation instead of forcing `canceled()` directly
      active_goal_->abort(std::make_shared<MoveToPose::Result>());

      active_goal_.reset();
    }

    RCLCPP_INFO(this->get_logger(), "[handle_goal] Received new goal!");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Handle cancel requests
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<MoveToPoseGoalHandle> goal_handle) {
    std::lock_guard<std::mutex> lock(goal_mutex_);

    if (goal_handle->is_active()) {
      RCLCPP_WARN(this->get_logger(),
                  "[handle_cancel] Moving goal to CANCELING.");
      return rclcpp_action::CancelResponse::ACCEPT;  // Moves the goal to
                                                     // CANCELING state
    }

    return rclcpp_action::CancelResponse::REJECT;
  }

  // Accept and process goal
  void handle_accepted(
      const std::shared_ptr<MoveToPoseGoalHandle> goal_handle) {
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      active_goal_ = goal_handle;
    }
    std::thread(&MoveToPoseServer::execute, this, goal_handle).detach();
  }

  // Execute goal and handle cancellation
  void execute(const std::shared_ptr<MoveToPoseGoalHandle> goal_handle) {
    auto result = std::make_shared<MoveToPose::Result>();
    auto feedback = std::make_shared<MoveToPose::Feedback>();

    rclcpp::Rate rate(1);
    for (int i = 1; i <= 5; ++i) {
      {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (!active_goal_ || active_goal_ != goal_handle) {
          RCLCPP_WARN(this->get_logger(),
                      "[execute] Goal was replaced. Stopping execution.");
          return;  // Exit thread safely
        }

        if (goal_handle->is_canceling()) {
          RCLCPP_WARN(this->get_logger(),
                      "[execute] Goal is canceling, finalizing as CANCELED.");

          // Ensure goal is still in CANCELING before calling canceled()
          if (goal_handle->is_active()) {
            goal_handle->canceled(result);
          }

          active_goal_.reset();
          return;
        }
      }

      feedback->current_pose = true;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(),
                  "[execute] Processing goal: %d sec elapsed", i);
      rate.sleep();
    }

    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (goal_handle->is_active()) {
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal processed successfully!");
      }
      active_goal_.reset();
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for the next goal...");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveToPoseServer>());
  rclcpp::shutdown();
  return 0;
}
