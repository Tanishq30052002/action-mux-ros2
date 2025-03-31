#ifndef ACTION_CLIENT_H
#define ACTION_CLIENT_H

#include <random>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

#include "calculator_msgs/action/calculator.hpp"

using Calculator = calculator_msgs::action::Calculator;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<Calculator>;
using namespace std::chrono_literals;
using namespace std::placeholders;

class CalculatorClient : public rclcpp::Node {
public:
  /**
   * @brief Construct a new Calculator Client object
   *
   */
  CalculatorClient();

private:
  ClientGoalHandle::SharedPtr client_goal_handle_;
  rclcpp_action::Client<Calculator>::SharedPtr client_;
  std::string topic_name_ = "/generic_topic";
  std::string detected_topic_type_;
  rclcpp::GenericSubscription::SharedPtr generic_subscriber_;
  rclcpp::TimerBase::SharedPtr type_check_timer_;

  /**
   * @brief Create a goal action object
   *
   * @param goal
   */
  void create_goal_action(Calculator::Goal &goal);

  /**
   * @brief
   *
   */
  void send_new_goal_to_server();

  /**
   * @brief
   *
   * @param goal_handle
   */
  void goal_response_callback(const ClientGoalHandle::SharedPtr &goal_handle);

  /**
   * @brief
   *
   * @param feedback
   */
  void feedback_callback(ClientGoalHandle::SharedPtr,
                         Calculator::Feedback::ConstSharedPtr feedback);

  /**
   * @brief
   *
   * @param result
   */
  void result_callback(const ClientGoalHandle::WrappedResult &result);

  void wait_for_topic() {
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

  void create_subscription() {
    if (!detected_topic_type_.empty()) {
      generic_subscriber_ = this->create_generic_subscription(
          topic_name_, detected_topic_type_, rclcpp::QoS(10),
          std::bind(&CalculatorClient::generic_subscriber_callback, this, _1));

      RCLCPP_INFO(this->get_logger(), "Subscribed to %s (Type: %s)",
                  topic_name_.c_str(), detected_topic_type_.c_str());
    }
  }

  void check_topic_status() {
    size_t num_publishers = this->count_publishers(topic_name_);

    if (num_publishers == 0) {
      RCLCPP_WARN(
          this->get_logger(),
          "No active publishers on topic '%s'. Resetting subscription...",
          topic_name_.c_str());

      generic_subscriber_.reset();  // Unsubscribe
      detected_topic_type_.clear(); // Reset detected type
      wait_for_topic();             // Wait for topic again
    }
  }

  void
  generic_subscriber_callback(std::shared_ptr<rclcpp::SerializedMessage> msg);
};

#endif // ACTION_CLIENT_H