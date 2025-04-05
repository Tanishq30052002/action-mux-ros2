#include "calculator_goal_publisher.h"

CalculatorGoalPublisher::CalculatorGoalPublisher(unsigned int time_interval_s)
    : Node("calculatorGoalPublisher"), time_interval_s_(time_interval_s) {
  calculator_goal_publisher_ =
      this->create_publisher<calculator_msgs::msg::CalculatorGoal>(
          "/calculator_goal", 10);

  timer_ = this->create_wall_timer(
      std::chrono::seconds(time_interval_s_),
      std::bind(&CalculatorGoalPublisher::calculatorGoalPublisher, this));
}

void CalculatorGoalPublisher::calculatorGoalPublisher() {
  auto msg = calculator_msgs::msg::CalculatorGoal();
  std::random_device rd;
  std::mt19937 mt(rd());

  std::uniform_real_distribution<float> value(-10.0, 10.0);
  msg.value_1 = value(mt);
  msg.value_2 = value(mt);

  std::vector<std::string> operations = {"add", "subtract", "multiply",
                                         "divide"};
  std::uniform_int_distribution<int> idx_operations(0, operations.size() - 1);
  msg.operation = operations[idx_operations(mt)];

  RCLCPP_INFO(this->get_logger(), "[calculatorGoalPublisher] Goal: %f %s %f",
              msg.value_1, msg.operation.c_str(), msg.value_2);
  calculator_goal_publisher_->publish(msg);
}
