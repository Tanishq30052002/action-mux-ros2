#include <calculator_server.h>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CalculateServer>());
  return 0;
}
