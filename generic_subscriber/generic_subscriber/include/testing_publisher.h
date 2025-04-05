#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

class TestingPublisher : public rclcpp::Node {
public:
  TestingPublisher(const std::string &topic_name, const int mode);

private:
  int mode_;
  std::string topic_name_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_bool_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_string_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_pose_;

  rclcpp::TimerBase::SharedPtr timer_;
};
