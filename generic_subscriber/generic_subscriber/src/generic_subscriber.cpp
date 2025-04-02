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

std::pair<std::string, std::string>
GenericSubscriber::getTopicTypeFromString(const std::string &topic_type_str) {
  std::string tmp = topic_type_str;
  size_t pos = tmp.find("/msg");
  if (pos != std::string::npos) {
    tmp.erase(pos, 4);
  }
  std::string::size_type split_at = tmp.find('/');
  if (split_at == std::string::npos) {
    throw std::runtime_error("invalid type specification");
  }
  auto topic_type = std::pair<std::string, std::string>(
      tmp.substr(0, split_at), tmp.substr(split_at + 1));

  return topic_type;
}

void GenericSubscriber::generic_subscriber_callback(
    std::shared_ptr<rclcpp::SerializedMessage> msg) {
  RCLCPP_INFO(this->get_logger(), "Serialized Message Received");

  std::pair<std::string, std::string> topic_type_pair =
      getTopicTypeFromString(detected_type_);

  RosMessage_Cpp ros_msg;
  const TypeInfo_Cpp *type_info = dynmsg::cpp::get_type_info(topic_type_pair);
  rcl_allocator_t msg_alloc = msg.get()->get_rcl_serialized_message().allocator;

  dynmsg::cpp::ros_message_with_typeinfo_init(type_info, &ros_msg, &msg_alloc);

  auto yaml_msg = dynmsg::cpp::message_to_yaml(ros_msg);
  auto string_msg = dynmsg::yaml_to_string(yaml_msg, true, true);
  RCLCPP_INFO(this->get_logger(), "Deserialized Msg: %s", string_msg.c_str());
}