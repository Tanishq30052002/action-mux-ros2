#include "calculator_client.h"

CalculatorClient::CalculatorClient() : Node("calculator_action_client") {
  client_ = rclcpp_action::create_client<Calculator>(this, "calculator");

  this->create_subscription<calculator_msgs::msg::CalculatorGoal>(
      "/calculator_goal", 10,
      std::bind(&CalculatorClient::calculator_goal_callback, this,
                std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "[CalculatorClient] Client is Ready !!!");
}

void CalculatorClient::calculator_goal_callback(
    const calculator_msgs::msg::CalculatorGoal::SharedPtr msg) {

  if (!client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(),
                 "[calculator_goal_callback] Action Server not available!");
    return;
  }

  RCLCPP_INFO(this->get_logger(),
              "[calculator_goal_callback] Received new goal!");
  // If an active goal exists, cancel it before sending a new one
  if (client_goal_handle_) {
    RCLCPP_WARN(
        this->get_logger(),
        "[calculator_goal_callback] Current Goal Active, Cancelling it");

    client_->async_cancel_goal(client_goal_handle_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  send_new_goal_to_server(msg);
}

void CalculatorClient::send_new_goal_to_server(
    const calculator_msgs::msg::CalculatorGoal::SharedPtr msg) {
  auto goal = Calculator::Goal();
  goal.goal.value_1 = msg->value_1;
  goal.goal.value_2 = msg->value_2;
  goal.goal.operation = msg->operation;

  auto send_goal_options = rclcpp_action::Client<Calculator>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&CalculatorClient::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&CalculatorClient::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&CalculatorClient::result_callback, this, _1);

  RCLCPP_INFO(this->get_logger(), "[send_new_goal_to_server] Sending new goal");
  auto future_goal_handle = client_->async_send_goal(goal, send_goal_options);

  // Handle future goal assignment
  std::thread([this, future_goal_handle]() {
    // Blocks until goal handle is ready
    auto goal_handle = future_goal_handle.get();
    if (goal_handle)
      client_goal_handle_ = goal_handle;
  }).detach();
}

void CalculatorClient::goal_response_callback(
    const ClientGoalHandle::SharedPtr &goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(),
                 "[goal_response_callback] Goal was rejected by server.");
  } else {
    RCLCPP_INFO(this->get_logger(), "[goal_response_callback] Goal accepted!");
  }
}

void CalculatorClient::feedback_callback(
    ClientGoalHandle::SharedPtr,
    Calculator::Feedback::ConstSharedPtr feedback) {
  float processing_time_left = feedback->time_remaining;
  RCLCPP_DEBUG(this->get_logger(),
               "[feedback_callback] Time left to process: %f",
               processing_time_left);
}

void CalculatorClient::result_callback(
    const ClientGoalHandle::WrappedResult &result) {
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->get_logger(),
                "[result_callback] Goal Succeeded, Result: %f",
                result.result.get()->result);
    client_goal_handle_ = nullptr;
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(this->get_logger(), "[result_callback] Goal Cancelled.");
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "[result_callback] Goal failed with code: %d",
                 static_cast<int>(result.code));
  }
}
