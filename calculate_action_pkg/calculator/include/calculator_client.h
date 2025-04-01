#ifndef CALCULATOR_CLIENT_H
#define CALCULATOR_CLIENT_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

#include "calculator_msgs/action/calculator.hpp"
#include "calculator_msgs/msg/calculator_goal.hpp"

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
  rclcpp::Subscription<calculator_msgs::msg::CalculatorGoal>::SharedPtr
      calculator_goal_subscriber_;

  /**
   * @brief
   *
   * @param msg
   */
  void calculator_goal_callback(
      const calculator_msgs::msg::CalculatorGoal::SharedPtr msg);

  /**
   * @brief
   *
   */
  void send_new_goal_to_server(
      const calculator_msgs::msg::CalculatorGoal::SharedPtr msg);

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
};

#endif // CALCULATOR_CLIENT_H