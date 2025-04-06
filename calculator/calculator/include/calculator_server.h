#ifndef CALCULATOR_SERVER_H
#define CALCULATOR_SERVER_H

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "calculator_msgs/action/calculator.hpp"
#include "visibility_control.h"

using Calculator = calculator_msgs::action::Calculator;
using ServerGoalHandle = rclcpp_action::ServerGoalHandle<Calculator>;
using namespace std::placeholders;

class CalculatorServer : public rclcpp::Node {
public:
  CalculatorServer(const float processing_time);

private:
  rclcpp_action::Server<Calculator>::SharedPtr server_;
  std::mutex goal_mutex_;
  float processing_time_;

  /**
   * @brief Handle goal requests, accept or reject them
   *
   * @param uuid
   * @param goal
   * @return rclcpp_action::GoalResponse
   */
  rclcpp_action::GoalResponse
  handleGoal(const rclcpp_action::GoalUUID &uuid,
             std::shared_ptr<const Calculator::Goal> goal);

  /**
   * @brief Handle cancel requests, accept or reject them, it request the
   * cancelling of the current goal, but not guarantee that it will be cancelled
   *
   * @param goal_handle the goal handle of the current goal
   * @return rclcpp_action::CancelResponse
   */
  rclcpp_action::CancelResponse
  handleCancel(const std::shared_ptr<ServerGoalHandle> goal_handle);

  /**
   * @brief Handle accepted goals, start a new thread to execute the goal
   *
   * @param goal_handle the goal handle of the current goal
   */
  void handleAccepted(const std::shared_ptr<ServerGoalHandle> goal_handle);

  /**
   * @brief Execute the goal, this function is called in a new thread. It
   * processes the current goal and in the mean time publishes feedback, until
   * the goal is reached or cancelled.
   *
   * @param goal_handle the goal handle of the current goal
   */
  void executeGoal(const std::shared_ptr<ServerGoalHandle> goal_handle);
};

#endif // CALCULATOR_SERVER_H