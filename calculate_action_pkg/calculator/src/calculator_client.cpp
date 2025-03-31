#include "calculator_client.h"

CalculateClient::CalculateClient() : Node("calculate_action_client") {
  client_ = rclcpp_action::create_client<Calculate>(this, "calculate");

  pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/goal_pose", 10,
      std::bind(&CalculateClient::goal_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "[constructor] Client is Ready !!!");
}

void CalculateClient::goal_callback(
    const geometry_msgs::msg::Pose::SharedPtr msg) {

  if (!client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(),
                 "[goal_callback] Action Server not available!");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "[goal_callback] Received new goal!");
  // If an active goal exists, cancel it before sending a new one
  if (client_goal_handle_) {
    RCLCPP_WARN(this->get_logger(),
                "[goal_callback] Current Goal Active, Cancelling it");

    client_->async_cancel_goal(client_goal_handle_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  send_new_goal_to_server();
}

void CalculateClient::create_goal_action(Calculate::Goal &goal) {
  std::random_device rd;
  std::mt19937 mt(rd());

  std::uniform_real_distribution<float> value(-10.0, 10.0);
  goal.value_1 = value(mt);
  goal.value_2 = value(mt);

  std::vector<std::string> operations = {"add", "subtract", "multiply",
                                         "divide"};
  std::uniform_int_distribution<int> idx_operations(0, operations.size() - 1);
  goal.operation = operations[idx_operations(mt)];
  RCLCPP_INFO(this->get_logger(), "[create_goal_msg] Goal: %f %s %f",
              goal.value_1, goal.operation.c_str(), goal.value_2);
}

void CalculateClient::send_new_goal_to_server() {
  auto goal = Calculate::Goal();
  create_goal_action(goal);

  auto send_goal_options = rclcpp_action::Client<Calculate>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&CalculateClient::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&CalculateClient::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&CalculateClient::result_callback, this, _1);

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

void CalculateClient::goal_response_callback(
    const ClientGoalHandle::SharedPtr &goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(),
                 "[goal_response_callback] Goal was rejected by server.");
  } else {
    RCLCPP_INFO(this->get_logger(), "[goal_response_callback] Goal accepted!");
  }
}

void CalculateClient::feedback_callback(
    ClientGoalHandle::SharedPtr, Calculate::Feedback::ConstSharedPtr feedback) {
  float processing_time_left = feedback->time_remaining;
  RCLCPP_DEBUG(this->get_logger(),
               "[feedback_callback] Time left to process: %f",
               processing_time_left);
}

void CalculateClient::result_callback(
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
