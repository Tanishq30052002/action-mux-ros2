#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "calculator_msgs/action/calculate.hpp"

using Calculate = calculator_msgs::action::Calculate;
using ServerGoalHandle = rclcpp_action::ServerGoalHandle<Calculate>;
using namespace std::chrono_literals;
using namespace std::placeholders;

class CalculateServer : public rclcpp::Node {
public:
  CalculateServer();

private:
  rclcpp_action::Server<Calculate>::SharedPtr server_;
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
              std::shared_ptr<const Calculate::Goal> goal);

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