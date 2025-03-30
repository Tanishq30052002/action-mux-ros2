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
  std::mutex goal_mutex_;

  // Handle new goals
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const MoveToPose::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "[handle_goal] Received new goal!");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Handle cancel requests
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<MoveToPoseGoalHandle> goal_handle) {
    std::lock_guard<std::mutex> lock(goal_mutex_);

    if (goal_handle->is_active()) {
      RCLCPP_WARN(this->get_logger(), "[handle_cancel] Cancelling Goal");
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    return rclcpp_action::CancelResponse::REJECT;
  }

  // Accept and process goal
  void handle_accepted(
      const std::shared_ptr<MoveToPoseGoalHandle> goal_handle) {
    std::thread(&MoveToPoseServer::execute, this, goal_handle).detach();
  }

  // Execute goal and handle cancellation
  void execute(const std::shared_ptr<MoveToPoseGoalHandle> goal_handle) {
    auto result = std::make_shared<MoveToPose::Result>();
    auto feedback = std::make_shared<MoveToPose::Feedback>();

    int frequency = 100;  // Hz
    rclcpp::Rate rate(frequency);
    for (int i = 1; i <= 5 * frequency && rclcpp::ok(); ++i) {
      if (goal_handle->is_canceling()) {
        feedback->current_pose = false;
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_WARN(this->get_logger(), "[execute] Goal Cancelled");
        return;
      }
      feedback->current_pose = true;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "[execute] Feedback Published: %d", i);
      rate.sleep();
    }
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (rclcpp::ok() && goal_handle->is_active()) {
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "[execute] Goal Process Successfully");
      }
    }
    if (!rclcpp::ok()) {
      RCLCPP_WARN(this->get_logger(),
                  "[execute] ROS is shutting down, stopping execution.");
      rclcpp::shutdown();
      return;
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveToPoseServer>());
  return 0;
}
