#ifndef CALCULATOR_CLIENT_H
#define CALCULATOR_CLIENT_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

#include "calculator_msgs/action/calculator.hpp"
#include "calculator_msgs/msg/calculator_goal.hpp"

using Calculator = calculator_msgs::action::Calculator;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<Calculator>;
using namespace std::placeholders;

class CalculatorClient : public rclcpp::Node {
public:
  /**
   * @brief Construct a new Calculator Client object, which get subvscribed to a
   * goal topic, also starts the send_goal_options threads for goal, feedback
   * and result callbacks.
   *
   */
  CalculatorClient();

private:
  ClientGoalHandle::SharedPtr client_goal_handle_;
  rclcpp_action::Client<Calculator>::SharedPtr client_;
  rclcpp::Subscription<calculator_msgs::msg::CalculatorGoal>::SharedPtr
      calculator_goal_subscriber_;
  rclcpp_action::Client<Calculator>::SendGoalOptions send_goal_options_;

  /**
   * @brief subscriber callback function, which is getting subscribed to the
   * goal and if prev goal is active it asks server to cancel it and send up
   * a new goal.
   *
   * @param msg ros2 msg which comes from a topic
   */
  void calculatorGoalCallback(
      const calculator_msgs::msg::CalculatorGoal::SharedPtr msg);

  /**
   * @brief it takes up the useful information from the msg and sends it to the
   * send it to the server. and also handles the future goal assignment.
   *
   * @param msg ros2 msg which comes from a topic
   */
  void sendNewGoalToServer(
      const calculator_msgs::msg::CalculatorGoal::SharedPtr msg);

  /**
   * @brief prints the goal status from the server, wheather it is accepted or
   * rejected
   *
   * @param goal_handle ptr to the goal handle
   */
  void goalResponseCallback(const ClientGoalHandle::SharedPtr &goal_handle);

  /**
   * @brief prints the feedback from the server in debug mode
   *
   * @param feedback is the feedback from the server which shows the time
   * remaining for the current goal to process.
   */
  void feedbackCallback(ClientGoalHandle::SharedPtr,
                        Calculator::Feedback::ConstSharedPtr feedback);

  /**
   * @brief prints the result from the server, if the goal is succeeded or
   * cancelled or none of the above.
   *
   * @param result is the result from the server which shows the result of the
   * goal
   */
  void resultCallback(const ClientGoalHandle::WrappedResult &result);
};

#endif // CALCULATOR_CLIENT_H