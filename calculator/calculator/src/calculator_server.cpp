#include <calculator_server.h>

CalculatorServer::CalculatorServer() : Node("calculator_action_server") {
  server_ = rclcpp_action::create_server<Calculator>(
      this, "calculator",
      std::bind(&CalculatorServer::handleGoal, this, _1, _2),
      std::bind(&CalculatorServer::handleCancel, this, _1),
      std::bind(&CalculatorServer::handleAccepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "[CalculatorServer] Server is Ready !!! ");
}

rclcpp_action::GoalResponse
CalculatorServer::handleGoal(const rclcpp_action::GoalUUID &uuid,
                             std::shared_ptr<const Calculator::Goal> goal) {
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "[handleGoal] Received new goal!");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CalculatorServer::handleCancel(
    const std::shared_ptr<ServerGoalHandle> goal_handle) {
  std::lock_guard<std::mutex> lock(goal_mutex_);

  if (goal_handle->is_active()) {
    RCLCPP_WARN(this->get_logger(), "[handleCancel] Cancelling Goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  return rclcpp_action::CancelResponse::REJECT;
}

void CalculatorServer::handleAccepted(
    const std::shared_ptr<ServerGoalHandle> goal_handle) {
  std::thread(&CalculatorServer::executeGoal, this, goal_handle).detach();
}

void CalculatorServer::executeGoal(
    const std::shared_ptr<ServerGoalHandle> goal_handle) {
  auto result = std::make_shared<Calculator::Result>();
  auto feedback = std::make_shared<Calculator::Feedback>();

  float frequency = 10.0;
  rclcpp::Rate rate(frequency);
  for (int i = 1; i <= processing_time_ * frequency && rclcpp::ok(); ++i) {
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_WARN(this->get_logger(), "[executeGoal] Goal Cancelled");
      return;
    }
    feedback->time_remaining = static_cast<float>(
        processing_time_ - static_cast<float>(i / frequency));
    goal_handle->publish_feedback(feedback);
    rate.sleep();
  }
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (rclcpp::ok() && goal_handle->is_active()) {
      auto goal = goal_handle->get_goal()->goal;
      if (goal.operation == "add") {
        result->result = goal.value_1 + goal.value_2;
      } else if (goal.operation == "subtract") {
        result->result = goal.value_1 - goal.value_2;
      } else if (goal.operation == "multiply") {
        result->result = goal.value_1 * goal.value_2;
      } else if (goal.operation == "divide") {
        if (goal.value_2)
          result->result = goal.value_1 / goal.value_2;
        else {
          result->result = 0;
          RCLCPP_ERROR(this->get_logger(),
                       "[executeGoal] Returning 0, as operation invalid");
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "[executeGoal] Invalid Operation");
        result->result = 0;
      }
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(),
                  "[executeGoal] Goal Process Successfully");
    }
  }
  if (!rclcpp::ok()) {
    RCLCPP_WARN(this->get_logger(),
                "[executeGoal] ROS is shutting down, stopping execution.");
    rclcpp::shutdown();
    return;
  }
}
