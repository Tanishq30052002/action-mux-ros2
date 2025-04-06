#ifndef CALCULATOR_GOAL_PUBLISHER_H
#define CALCULATOR_GOAL_PUBLISHER_H

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
   * @brief Construct a new Calculator Goal Publisher object, which creates a
   * timer which publishes the goal on a topic at a certain time interval.
   *
   * @param time_interval_s time interval in seconds
   * @note The time interval is the time between two consecutive goal
   */
  CalculatorGoalPublisher(unsigned int time_interval_s);

private:
  /**
   * @brief This function is called by the timer and publishes a new goal on the
   * topic. It generates random values for the goal and publishes it.
   *
   */
  void calculatorGoalPublisher();

  rclcpp::Publisher<calculator_msgs::msg::CalculatorGoal>::SharedPtr
      calculator_goal_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  unsigned int time_interval_s_ = 1;
};

#endif // CALCULATOR_GOAL_PUBLISHER_H