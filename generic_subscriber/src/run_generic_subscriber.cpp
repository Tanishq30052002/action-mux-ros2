#include "generic_subscriber.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GenericSubscriber>());
  rclcpp::shutdown();
  return 0;
}