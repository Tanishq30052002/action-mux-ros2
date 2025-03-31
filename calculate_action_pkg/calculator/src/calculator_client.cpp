#include "calculator_client.h"

CalculatorClient::CalculatorClient() : Node("calculator_action_client") {
  client_ = rclcpp_action::create_client<Calculator>(this, "calculator");
  RCLCPP_INFO(this->get_logger(), "[constructor] Client is Ready !!!");

  wait_for_topic();

  // Timer to check for active publishers and type changes
  type_check_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&CalculatorClient::check_topic_status, this));
}

void CalculatorClient::generic_subscriber_callback(
    std::shared_ptr<rclcpp::SerializedMessage> msg) {
  RCLCPP_INFO(this->get_logger(),
              "[generic_subscriber_callback] Received a serialized message!");
  print_topic_msg(msg);

  if (!client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(),
                 "[generic_subscriber_callback] Action Server not available!");
    return;
  }

  // If an active goal exists, cancel it before sending a new one
  if (client_goal_handle_) {
    RCLCPP_WARN(
        this->get_logger(),
        "[generic_subscriber_callback] Current Goal Active, Cancelling it");

    client_->async_cancel_goal(client_goal_handle_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  send_new_goal_to_server();
}

void CalculatorClient::create_goal_action(Calculator::Goal &goal) {
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

void CalculatorClient::send_new_goal_to_server() {
  auto goal = Calculator::Goal();
  create_goal_action(goal);

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

void CalculatorClient::wait_for_topic() {
  while (rclcpp::ok()) {
    auto topics = this->get_topic_names_and_types();
    auto it = topics.find(topic_name_);

    if (it == topics.end() || it->second.empty()) {
      RCLCPP_WARN(this->get_logger(), "Topic '%s' not found. Waiting...",
                  topic_name_.c_str());
      rclcpp::sleep_for(std::chrono::milliseconds(500)); // Retry delay
      continue;
    }

    RCLCPP_INFO(this->get_logger(), "Topic '%s' detected!",
                topic_name_.c_str());
    detected_topic_type_ = it->second[0];
    RCLCPP_INFO(this->get_logger(), "Detected topic type: %s",
                detected_topic_type_.c_str());

    create_subscription();
    break; // Exit loop once subscription is created
  }
}

void CalculatorClient::create_subscription() {
  if (!detected_topic_type_.empty()) {
    generic_subscriber_ = this->create_generic_subscription(
        topic_name_, detected_topic_type_, rclcpp::QoS(10),
        std::bind(&CalculatorClient::generic_subscriber_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s (Type: %s)",
                topic_name_.c_str(), detected_topic_type_.c_str());
  }
}

void CalculatorClient::check_topic_status() {
  size_t num_publishers = this->count_publishers(topic_name_);

  if (num_publishers == 0) {
    RCLCPP_WARN(this->get_logger(),
                "No active publishers on topic '%s'. Resetting subscription...",
                topic_name_.c_str());

    generic_subscriber_.reset();  // Unsubscribe
    detected_topic_type_.clear(); // Reset detected type
    wait_for_topic();             // Wait for topic again
  }
}

void CalculatorClient::print_topic_msg(
    const std::shared_ptr<rclcpp::SerializedMessage> &msg) {}