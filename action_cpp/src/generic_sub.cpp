#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "std_msgs/msg/string.hpp"

class GenericSubscriber : public rclcpp::Node {
 public:
  GenericSubscriber(const std::string &topic_name)
      : Node("generic_subscriber"), topic_name_(topic_name) {
    // Start monitoring the topic
    wait_for_topic();

    // Timer to check for active publishers and type changes
    type_check_timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&GenericSubscriber::check_topic_status, this));
  }

 private:
  void wait_for_topic() {
    while (rclcpp::ok()) {
      auto topics = this->get_topic_names_and_types();
      auto it = topics.find(topic_name_);

      if (it == topics.end() || it->second.empty()) {
        RCLCPP_WARN(this->get_logger(), "Topic '%s' not found. Waiting...",
                    topic_name_.c_str());
        rclcpp::sleep_for(std::chrono::milliseconds(500));  // Retry delay
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "Topic '%s' detected!",
                  topic_name_.c_str());
      detected_type_ = it->second[0];
      RCLCPP_INFO(this->get_logger(), "Detected topic type: %s",
                  detected_type_.c_str());

      create_subscription();
      break;  // Exit loop once subscription is created
    }
  }

  void create_subscription() {
    if (!detected_type_.empty()) {
      generic_subscriber_ = this->create_generic_subscription(
          topic_name_, detected_type_, rclcpp::QoS(10),
          std::bind(&GenericSubscriber::on_message_received, this,
                    std::placeholders::_1));

      RCLCPP_INFO(this->get_logger(), "Subscribed to %s (Type: %s)",
                  topic_name_.c_str(), detected_type_.c_str());
    }
  }

  void check_topic_status() {
    size_t num_publishers = this->count_publishers(topic_name_);

    if (num_publishers == 0) {
      RCLCPP_WARN(
          this->get_logger(),
          "No active publishers on topic '%s'. Resetting subscription...",
          topic_name_.c_str());

      generic_subscriber_.reset();  // Unsubscribe
      detected_type_.clear();       // Reset detected type
      wait_for_topic();             // Wait for topic again
    }
  }

  void on_message_received(std::shared_ptr<rclcpp::SerializedMessage> msg) {
    RCLCPP_INFO(this->get_logger(),
                "[generic_subscription] Received a serialized message!");
  }

  std::string topic_name_;
  std::string detected_type_;
  rclcpp::GenericSubscription::SharedPtr generic_subscriber_;
  rclcpp::TimerBase::SharedPtr type_check_timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GenericSubscriber>("/goal_pose");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
