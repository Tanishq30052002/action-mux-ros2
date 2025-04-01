#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <string>

#include "calculator_msgs/msg/calculator_goal.hpp"
#include "rclcpp/rclcpp.hpp"

class CalculatorGoalPublisher : public rclcpp::Node {
public:
  /**
   * @brief Construct a new Calculator Goal Publisher object
   *
   * @param time_interval_s
   */
  CalculatorGoalPublisher(unsigned int time_interval_s);

private:
  /**
   * @brief
   *
   */
  void calculator_goal_publisher();

  rclcpp::Publisher<calculator_msgs::msg::CalculatorGoal>::SharedPtr
      calculator_goal_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  unsigned int time_interval_s_ = 1;
};
