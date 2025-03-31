#ifndef ACTION_CLIENT_H
#define ACTION_CLIENT_H

#include <geometry_msgs/msg/pose.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

#include "calculate_action_pkg/action/calculate.hpp"

using Calculate = calculate_action_pkg::action::Calculate;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<Calculate>;
using namespace std::chrono_literals;
using namespace std::placeholders;

class CalculateClient : public rclcpp::Node {
public:
  /**
   * @brief Construct a new Calculate Client object
   *
   */
  CalculateClient();

private:
  ClientGoalHandle::SharedPtr client_goal_handle_;
  rclcpp_action::Client<Calculate>::SharedPtr client_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber_;

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
  void create_goal_action(Calculate::Goal &goal);

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
                         Calculate::Feedback::ConstSharedPtr feedback);

  /**
   * @brief
   *
   * @param result
   */
  void result_callback(const ClientGoalHandle::WrappedResult &result);
};

#endif // ACTION_CLIENT_H