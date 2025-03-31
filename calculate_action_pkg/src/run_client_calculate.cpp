#include "calculator_client.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CalculateClient>());
  rclcpp::shutdown();
  return 0;
}