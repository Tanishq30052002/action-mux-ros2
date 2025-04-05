#include "generic_subscriber.h"

GenericSubscriber::GenericSubscriber() : Node("generic_subscriber") {
  waitForTopic();

  // Timer to check for active publishers and type changes
  type_check_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&GenericSubscriber::checkTopicStatus, this));
}

void GenericSubscriber::checkTopicStatus() {
  if (this->count_publishers(topic_name_))
    return;
  RCLCPP_WARN(this->get_logger(),
              "[checkTopicStatus] No active publishers on topic '%s'. "
              "Resetting subscription...",
              topic_name_.c_str());

  generic_subscriber_.reset(); // Unsubscribe
  detected_type_.clear();      // Reset detected type
  waitForTopic();              // Wait for topic again
  return;
}

void GenericSubscriber::waitForTopic() {
  while (rclcpp::ok()) {
    auto topics = this->get_topic_names_and_types();
    auto it = topics.find(topic_name_);

    if (it == topics.end() || it->second.empty()) {
      RCLCPP_WARN(this->get_logger(), "Topic '%s' not found. Waiting...",
                  topic_name_.c_str());
      rclcpp::sleep_for(std::chrono::milliseconds(500)); // Retry delay
      continue;
    }

    RCLCPP_INFO(this->get_logger(), "[waitForTopic] Topic '%s' detected!",
                topic_name_.c_str());
    detected_type_ = it->second[0];
    RCLCPP_INFO(this->get_logger(), "[waitForTopic] Detected topic type: %s",
                detected_type_.c_str());

    createSubscription();
    break; // Exit loop once subscription is created
  }
}

void GenericSubscriber::createSubscription() {
  if (!detected_type_.empty()) {
    generic_subscriber_ = this->create_generic_subscription(
        topic_name_, detected_type_, rclcpp::QoS(10),
        std::bind(&GenericSubscriber::genericSubscriberCallback, this, _1));

    RCLCPP_INFO(this->get_logger(),
                "[createSubscription] Subscribed to %s (Type: %s)",
                topic_name_.c_str(), detected_type_.c_str());
  }
}

void GenericSubscriber::getTopicTypeFromString(
    InterfaceTypeName &topic_type_pair) {
  std::string tmp = detected_type_;
  size_t pos = tmp.find("/msg");
  if (pos != std::string::npos) {
    tmp.erase(pos, 4);
  }
  std::string::size_type split_at = tmp.find('/');
  if (split_at == std::string::npos) {
    throw std::runtime_error(
        "[getTopicTypeFromString] invalid type specification");
  }
  topic_type_pair.first = tmp.substr(0, split_at);
  topic_type_pair.second = tmp.substr(split_at + 1);
}

void GenericSubscriber::genericSubscriberCallback(
    std::shared_ptr<rclcpp::SerializedMessage> msg) {
  InterfaceTypeName interface_type;
  getTopicTypeFromString(interface_type);

  RosMessage_Cpp ros_msg;
  ros_msg.type_info = dynmsg::cpp::get_type_info(interface_type);
  rcl_allocator_t *msg_alloc =
      &msg.get()->get_rcl_serialized_message().allocator;

  ros_msg.data = static_cast<uint8_t *>(
      msg_alloc->allocate(ros_msg.type_info->size_of_, msg_alloc->state));

  ros_msg.type_info->init_function(
      ros_msg.data, rosidl_runtime_cpp::MessageInitialization::ZERO);

  auto yaml_msg = dynmsg::cpp::message_to_yaml(ros_msg);
  auto string_msg = dynmsg::yaml_to_string(yaml_msg, true, false);
  RCLCPP_INFO(this->get_logger(),
              "[genericSubscriberCallback]\nTopic: %s\nDetected Type: "
              "%s\nROS2 Message:\n%s",
              topic_name_.c_str(), detected_type_.c_str(), string_msg.c_str());

  ros_msg.type_info->fini_function(ros_msg.data);
  msg_alloc->deallocate(ros_msg.data, msg_alloc->state);
}