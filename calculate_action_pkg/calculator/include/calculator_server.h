#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "calculator_msgs/action/calculator.hpp"
#include "visibility_control.h"

using Calculator = calculator_msgs::action::Calculator;
using ServerGoalHandle = rclcpp_action::ServerGoalHandle<Calculator>;
using namespace std::chrono_literals;
using namespace std::placeholders;

class CalculatorServer : public rclcpp::Node {
public:
  CalculatorServer();

private:
  rclcpp_action::Server<Calculator>::SharedPtr server_;
  std::mutex goal_mutex_;

  /**
   * @brief Handle new goals
   *
   * @param uuid
   * @param goal
   * @return rclcpp_action::GoalResponse
   */
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Calculator::Goal> goal);

  /**
   * @brief Handle cancel requests
   *
   * @param goal_handle
   * @return rclcpp_action::CancelResponse
   */
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<ServerGoalHandle> goal_handle);

  /**
   * @brief Accept and process goal
   *
   * @param goal_handle
   */
  void handle_accepted(const std::shared_ptr<ServerGoalHandle> goal_handle);

  /**
   * @brief Execute goal and handle cancellation
   *
   * @param goal_handle
   */
  void execute(const std::shared_ptr<ServerGoalHandle> goal_handle);
};