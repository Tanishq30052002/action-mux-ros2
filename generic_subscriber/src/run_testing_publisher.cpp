#include "testing_publisher.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  if (argc != 2) {
    std::cout
        << "Usage:\nros2 run generic_subscriber run_testing_publisher "
           "<MODE>\nMODE: 1 - For publishing at std_msgs::msg::String\nMODE: 2 "
           "- For publishing at std_msgs::msg::Bool\nMODE: 3 - For publishing "
           "at geometry_msgs::msg::Twist\nMODE: 4 - For publishing at "
           "geometry_msgs::msg::Pose\n";
    return 0;
  }
  int mode = std::atoi(argv[1]);
  std::cout << mode;
  rclcpp::spin(std::make_shared<TestingPublisher>("/generic_topic", mode));
  rclcpp::shutdown();
  return 0;
}