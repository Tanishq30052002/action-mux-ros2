#include "calculator_client.h"

CalculatorClient::CalculatorClient() : Node("calculator_action_client") {
  client_ = rclcpp_action::create_client<Calculator>(this, "calculator");

  calculator_goal_subscriber_ =
      this->create_subscription<calculator_msgs::msg::CalculatorGoal>(
          "/calculator_goal", 10,
          std::bind(&CalculatorClient::calculatorGoalCallback, this, _1));

  RCLCPP_INFO(this->get_logger(), "[CalculatorClient] Client is Ready !!!");
}

void CalculatorClient::calculatorGoalCallback(
    const calculator_msgs::msg::CalculatorGoal::SharedPtr msg) {

  if (!client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(),
                 "[calculatorGoalCallback] Action Server not available!");
    return;
  }

  RCLCPP_INFO(this->get_logger(),
              "[calculatorGoalCallback] Received new goal!");
  // If an active goal exists, cancel it before sending a new one
  if (client_goal_handle_) {
    RCLCPP_WARN(this->get_logger(),
                "[calculatorGoalCallback] Current Goal Active, Cancelling it");

    client_->async_cancel_goal(client_goal_handle_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  sendNewGoalToServer(msg);
}

void CalculatorClient::sendNewGoalToServer(
    const calculator_msgs::msg::CalculatorGoal::SharedPtr msg) {
  auto goal = Calculator::Goal();
  goal.goal.value_1 = msg->value_1;
  goal.goal.value_2 = msg->value_2;
  goal.goal.operation = msg->operation;

  auto send_goal_options = rclcpp_action::Client<Calculator>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&CalculatorClient::goalResponseCallback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&CalculatorClient::feedbackCallback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&CalculatorClient::resultCallback, this, _1);

  RCLCPP_INFO(this->get_logger(), "[sendNewGoalToServer] Sending new goal");
  auto future_goal_handle = client_->async_send_goal(goal, send_goal_options);

  // Handle future goal assignment
  std::thread([this, future_goal_handle]() {
    // Blocks until goal handle is ready
    auto goal_handle = future_goal_handle.get();
    if (goal_handle)
      client_goal_handle_ = goal_handle;
  }).detach();
}

void CalculatorClient::goalResponseCallback(
    const ClientGoalHandle::SharedPtr &goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(),
                 "[goalResponseCallback] Goal was rejected by server.");
  } else {
    RCLCPP_INFO(this->get_logger(), "[goalResponseCallback] Goal accepted!");
  }
}

void CalculatorClient::feedbackCallback(
    ClientGoalHandle::SharedPtr,
    Calculator::Feedback::ConstSharedPtr feedback) {
  float processing_time_left = feedback->time_remaining;
  RCLCPP_DEBUG(this->get_logger(),
               "[feedbackCallback] Time left to process: %f",
               processing_time_left);
}

void CalculatorClient::resultCallback(
    const ClientGoalHandle::WrappedResult &result) {
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->get_logger(),
                "[resultCallback] Goal Succeeded, Result: %f",
                result.result.get()->result);
    client_goal_handle_ = nullptr;
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(this->get_logger(), "[resultCallback] Goal Cancelled.");
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "[resultCallback] Goal failed with code: %d",
                 static_cast<int>(result.code));
  }
}
