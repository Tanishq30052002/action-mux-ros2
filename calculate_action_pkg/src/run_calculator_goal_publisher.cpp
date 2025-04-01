#include "calculator_goal_publisher.h"

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <time_interval_s>" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CalculatorGoalPublisher>(
      static_cast<unsigned int>(std::stoi(argv[1]))));
  rclcpp::shutdown();
  return 0;
}