#include "generic_subscriber.h"

GenericSubscriber::GenericSubscriber() : Node("generic_subscriber") {
  wait_for_topic();

  // Timer to check for active publishers and type changes
  type_check_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&GenericSubscriber::check_topic_status, this));
}

void GenericSubscriber::check_topic_status() {
  if (this->count_publishers(topic_name_))
    return;
  RCLCPP_WARN(this->get_logger(),
              "No active publishers on topic '%s'. Resetting subscription...",
              topic_name_.c_str());

  generic_subscriber_.reset(); // Unsubscribe
  detected_type_.clear();      // Reset detected type
  wait_for_topic();            // Wait for topic again
  return;
}

void GenericSubscriber::wait_for_topic() {
  while (rclcpp::ok()) {
    auto topics = this->get_topic_names_and_types();
    auto it = topics.find(topic_name_);

    if (it == topics.end() || it->second.empty()) {
      RCLCPP_WARN(this->get_logger(), "Topic '%s' not found. Waiting...",
                  topic_name_.c_str());
      rclcpp::sleep_for(std::chrono::milliseconds(500)); // Retry delay
      continue;
    }

    RCLCPP_INFO(this->get_logger(), "Topic '%s' detected!",
                topic_name_.c_str());
    detected_type_ = it->second[0];
    RCLCPP_INFO(this->get_logger(), "Detected topic type: %s",
                detected_type_.c_str());

    create_subscription();
    break; // Exit loop once subscription is created
  }
}

void GenericSubscriber::create_subscription() {
  if (!detected_type_.empty()) {
    generic_subscriber_ = this->create_generic_subscription(
        topic_name_, detected_type_, rclcpp::QoS(10),
        std::bind(&GenericSubscriber::generic_subscriber_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s (Type: %s)",
                topic_name_.c_str(), detected_type_.c_str());
  }
}
std::string removeMsgFromTopicType(const std::string &topic_type) {
  std::string result = topic_type;
  size_t pos = result.find("/msg");
  if (pos != std::string::npos) {
    result.erase(pos, 4);
  }
  return result;
}

void GenericSubscriber::generic_subscriber_callback(
    std::shared_ptr<rclcpp::SerializedMessage> msg) {
  RCLCPP_INFO(this->get_logger(), "Serialized msg received");

  InterfaceTypeName topic_type_name =
      get_topic_type_from_string_type(removeMsgFromTopicType(detected_type_));
  const rosidl_message_type_support_t *ts = get_type_support(topic_type_name);
  auto deserializer = rclcpp::SerializationBase(ts);

  RosMessage_Cpp result;
  deserializer.deserialize_message(msg.get(), &result);
  auto yaml_result = dynmsg::cpp::message_to_yaml(result);

  RCLCPP_INFO(this->get_logger(), "Deserialized Msg in YAML Format: %s",
              yaml_result);
}