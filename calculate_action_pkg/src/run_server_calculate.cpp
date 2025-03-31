#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "calculate_action_pkg/action/calculate.hpp"

using Calculate = calculate_action_pkg::action::Calculate;
using ServerGoalHandle = rclcpp_action::ServerGoalHandle<Calculate>;
using namespace std::chrono_literals;
using namespace std::placeholders;

class CalculateServer : public rclcpp::Node {
public:
  CalculateServer() : Node("calculate_action_server") {
    server_ = rclcpp_action::create_server<Calculate>(
        this, "calculate",
        std::bind(&CalculateServer::handle_goal, this, _1, _2),
        std::bind(&CalculateServer::handle_cancel, this, _1),
        std::bind(&CalculateServer::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "[constructor] Server is Ready !!! ");
  }

private:
  rclcpp_action::Server<Calculate>::SharedPtr server_;
  std::mutex goal_mutex_;

  // Handle new goals
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Calculate::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "[handle_goal] Received new goal!");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Handle cancel requests
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<ServerGoalHandle> goal_handle) {
    std::lock_guard<std::mutex> lock(goal_mutex_);

    if (goal_handle->is_active()) {
      RCLCPP_WARN(this->get_logger(), "[handle_cancel] Cancelling Goal");
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    return rclcpp_action::CancelResponse::REJECT;
  }

  // Accept and process goal
  void handle_accepted(const std::shared_ptr<ServerGoalHandle> goal_handle) {
    std::thread(&CalculateServer::execute, this, goal_handle).detach();
  }

  // Execute goal and handle cancellation
  void execute(const std::shared_ptr<ServerGoalHandle> goal_handle) {
    auto result = std::make_shared<Calculate::Result>();
    auto feedback = std::make_shared<Calculate::Feedback>();

    int frequency = 100; // Hz
    rclcpp::Rate rate(frequency);
    for (int i = 1; i <= 5 * frequency && rclcpp::ok(); ++i) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_WARN(this->get_logger(), "[execute] Goal Cancelled");
        return;
      }
      feedback->time_remaining =
          static_cast<float>(5.0 - static_cast<float>(i / 100.0));
      goal_handle->publish_feedback(feedback);
      rate.sleep();
    }
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (rclcpp::ok() && goal_handle->is_active()) {
        auto goal = goal_handle->get_goal();
        if (goal->operation == "add") {
          result->result = goal->value_1 + goal->value_2;
        } else if (goal->operation == "subtract") {
          result->result = goal->value_1 - goal->value_2;
        } else if (goal->operation == "multiply") {
          result->result = goal->value_1 * goal->value_2;
        } else if (goal->operation == "divide") {
          result->result = goal->value_1 / goal->value_2;
        } else {
          RCLCPP_ERROR(this->get_logger(), "[execute] Invalid Operation");
          result->result = 0;
        }
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
  rclcpp::spin(std::make_shared<CalculateServer>());
  return 0;
}
