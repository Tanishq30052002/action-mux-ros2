#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

class TestingPublisher : public rclcpp::Node {
public:
  /**
   * @brief Construct a new Testing Publisher object. Based on the mode, it will
   * create a publisher which publishes at 1Hz.
   *
   * @param topic_name the name of the topic to publish to
   * @param mode the mode of the publisher. 1 for string, 2 for bool, 3 for
   * twist, 4 for pose
   *
   */
  TestingPublisher(const std::string &topic_name, const int mode);

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_bool_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_string_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_pose_;

  rclcpp::TimerBase::SharedPtr timer_;
};
