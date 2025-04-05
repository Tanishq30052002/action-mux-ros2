#include "generic_subscriber.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GenericSubscriber>("/generic_topic"));
  rclcpp::shutdown();
  return 0;
}