#include "testing_publisher.h"

TestingPublisher::TestingPublisher(const std::string &topic_name,
                                   const int mode)
    : Node("testing_publisher") {
  switch (mode) {
  case 1: {
    pub_string_ = this->create_publisher<std_msgs::msg::String>(
        topic_name, rclcpp::QoS(10));
    timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
      auto msg = std_msgs::msg::String();
      msg.data = "Hello World !!!";
      pub_string_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Published");
    });
    break;
  }
  case 2: {
    pub_bool_ = this->create_publisher<std_msgs::msg::Bool>(topic_name,
                                                            rclcpp::QoS(10));
    timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
      auto msg = std_msgs::msg::Bool();
      msg.data = true;
      pub_bool_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Published");
    });
    break;
  }
  case 3: {
    pub_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(
        topic_name, rclcpp::QoS(10));
    timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
      auto msg = geometry_msgs::msg::Twist();
      msg.angular.x = 1.0;
      msg.angular.y = 2.0;
      msg.angular.z = 3.0;
      msg.linear.x = 4.0;
      msg.linear.y = 5.0;
      msg.linear.z = 6.0;
      pub_twist_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Published");
    });
    break;
  }
  case 4: {
    pub_pose_ = this->create_publisher<geometry_msgs::msg::Pose>(
        topic_name, rclcpp::QoS(10));
    timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
      auto msg = geometry_msgs::msg::Pose();
      msg.position.x = 1.0;
      msg.position.y = 2.0;
      msg.position.z = 3.0;
      msg.orientation.w = 4.0;
      msg.orientation.x = 5.0;
      msg.orientation.y = 6.0;
      msg.orientation.z = 7.0;
      pub_pose_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Published");
    });
    break;
  }
  default:
    RCLCPP_ERROR(this->get_logger(), "Invalid mode. Use 1-4.");
    rclcpp::shutdown();
  }
}
