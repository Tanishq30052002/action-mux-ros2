#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

// Include possible message types
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"

class GenericSubscriber : public rclcpp::Node {
 public:
  GenericSubscriber(const std::string &topic_name)
      : Node("generic_subscriber"), topic_name_(topic_name) {
    // Detect topic type at runtime
    detect_topic_type();

    // Subscribe using a generic subscriber
    generic_subscriber_ = this->create_generic_subscription(
        topic_name_, topic_type_, rclcpp::QoS(10),
        std::bind(&GenericSubscriber::on_message_received, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s (Type: %s)",
                topic_name_.c_str(), topic_type_.c_str());
  }

 private:
  void detect_topic_type() {
    // Get all active topics and their types
    auto topics = this->get_topic_names_and_types();
    auto it = topics.find(topic_name_);

    if (it != topics.end() && !it->second.empty()) {
      topic_type_ = it->second[0];  // Use the first available type
      RCLCPP_INFO(this->get_logger(), "Detected topic type: %s",
                  topic_type_.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Topic '%s' not found or has no type!",
                   topic_name_.c_str());
      rclcpp::shutdown();
    }
  }

  void on_message_received(std::shared_ptr<rclcpp::SerializedMessage> msg) {
    RCLCPP_INFO(this->get_logger(),
                "[generic_subscription] Received a serialized message!");

    // Deserialize dynamically based on detected type
    // if (topic_type_ == "std_msgs/msg/String") {
    //   deserialize_and_print<std_msgs::msg::String>(msg);
    // } else if (topic_type_ == "geometry_msgs/msg/PoseStamped") {
    //   deserialize_and_print<geometry_msgs::msg::PoseStamped>(msg);
    // } else {
    //   RCLCPP_WARN(this->get_logger(), "Unsupported message type: %s",
    //               topic_type_.c_str());
    // }
  }

  template <typename MessageType>
  void deserialize_and_print(std::shared_ptr<rclcpp::SerializedMessage> msg) {
    MessageType deserialized_msg;
    rclcpp::Serialization<MessageType> serializer;
    serializer.deserialize_message(msg.get(), &deserialized_msg);

    // Print message (customize this for each message type)
    if constexpr (std::is_same<MessageType, std_msgs::msg::String>::value) {
      RCLCPP_INFO(this->get_logger(), "Deserialized String: %s",
                  deserialized_msg.data.c_str());
    } else if constexpr (std::is_same<MessageType,
                                      geometry_msgs::msg::PoseStamped>::value) {
      RCLCPP_INFO(this->get_logger(),
                  "Deserialized PoseStamped: Position(x: %f, y: %f, z: %f)",
                  deserialized_msg.pose.position.x,
                  deserialized_msg.pose.position.y,
                  deserialized_msg.pose.position.z);
    }
  }

  std::string topic_name_;
  std::string topic_type_;
  rclcpp::GenericSubscription::SharedPtr generic_subscriber_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <topic_name>\n";
    return 1;
  }

  auto node = std::make_shared<GenericSubscriber>(argv[1]);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
