#ifndef ACTION_CLIENT_H
#define ACTION_CLIENT_H

#include <geometry_msgs/msg/pose.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
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

  /**
   * @brief
   *
   * @param msg
   */
  void goal_callback(const geometry_msgs::msg::Pose::SharedPtr msg);

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
};

#endif // ACTION_CLIENT_H