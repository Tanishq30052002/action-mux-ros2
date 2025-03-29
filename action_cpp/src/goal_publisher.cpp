#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

class GoalPublisher : public rclcpp::Node {
 public:
  GoalPublisher() : Node("goal_publisher") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Pose>("/goal_pose", 10);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(2), std::bind(&GoalPublisher::publish_goal, this));
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_ = 0;

  void publish_goal() {
    auto message = geometry_msgs::msg::Pose();
    
    // Example goal: Increasing x-position, fixed y and z
    message.position.x = count_ * 1.0;  // Increment x by 1 each time
    message.position.y = 0.0;
    message.position.z = 0.0;

    // Orientation (Quaternion) - No rotation
    message.orientation.x = 0.0;
    message.orientation.y = 0.0;
    message.orientation.z = 0.0;
    message.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(),
                "Publishing Goal Pose: [x: %f, y: %f, z: %f]",
                message.position.x, message.position.y, message.position.z);

    publisher_->publish(message);
    count_++;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalPublisher>());
  rclcpp::shutdown();
  return 0;
}
