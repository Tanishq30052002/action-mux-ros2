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
  std::string topic_name_ = "/generic_topic";
  std::string detected_topic_type_;
  rclcpp::GenericSubscription::SharedPtr generic_subscriber_;
  rclcpp::TimerBase::SharedPtr type_check_timer_;
  ClientGoalHandle::SharedPtr client_goal_handle_;
  rclcpp_action::Client<Calculator>::SharedPtr client_;

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

  /**
   * @brief
   *
   */
  void wait_for_topic();

  /**
   * @brief Create a subscription object
   *
   */
  void create_subscription();

  /**
   * @brief
   *
   */
  void check_topic_status();

  /**
   * @brief
   *
   * @param msg
   */
  void
  generic_subscriber_callback(std::shared_ptr<rclcpp::SerializedMessage> msg);

  /**
   * @brief
   *
   * @param msg
   */
  void print_topic_msg(const std::shared_ptr<rclcpp::SerializedMessage> &msg);
};

#endif // ACTION_CLIENT_H