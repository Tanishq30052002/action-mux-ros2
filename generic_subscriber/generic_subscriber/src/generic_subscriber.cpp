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
              "[check_topic_status] No active publishers on topic '%s'. "
              "Resetting subscription...",
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

    RCLCPP_INFO(this->get_logger(), "[wait_for_topic] Topic '%s' detected!",
                topic_name_.c_str());
    detected_type_ = it->second[0];
    RCLCPP_INFO(this->get_logger(), "[wait_for_topic] Detected topic type: %s",
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

    RCLCPP_INFO(this->get_logger(),
                "[create_subscription] Subscribed to %s (Type: %s)",
                topic_name_.c_str(), detected_type_.c_str());
  }
}

void GenericSubscriber::getTopicTypeFromString(
    std::pair<std::string, std::string> &topic_type_pair) {
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

typedef const rosidl_message_type_support_t *(*get_message_ts_func)();
using TypeInfo_Cpp = rosidl_typesupport_introspection_cpp::MessageMembers;

void GenericSubscriber::generic_subscriber_callback(
    std::shared_ptr<rclcpp::SerializedMessage> msg) {

  std::pair<std::string, std::string> interface_type;
  getTopicTypeFromString(interface_type);
  const std::string pkg_name = interface_type.first;
  const std::string msg_name = interface_type.second;

  std::string ts_lib_name =
      "lib" + pkg_name + "__rosidl_typesupport_introspection_cpp.so";

  RCUTILS_LOG_DEBUG_NAMED("dynmsg",
                          "Loading C++ introspection type support library %s",
                          ts_lib_name.c_str());
  void *introspection_type_support_lib = dlopen(ts_lib_name.c_str(), RTLD_LAZY);
  if (nullptr == introspection_type_support_lib) {
    RCUTILS_LOG_ERROR_NAMED(
        "dynmsg", "failed to load C++ introspection type support library: %s",
        dlerror());
  }

  std::string ts_func_name = "_ZN36rosidl_typesupport_introspection_cpp31get_"
                             "message_type_support_handleIN" +
                             std::to_string(pkg_name.length()) + pkg_name +
                             "3msg" + std::to_string(msg_name.length() + 1) +
                             msg_name +
                             "_ISaIvEEEEEPK29rosidl_message_type_support_tv";
  RCUTILS_LOG_DEBUG_NAMED("dynmsg", "Loading C++ type support function %s",
                          ts_func_name.c_str());

  get_message_ts_func introspection_type_support_handle_func =
      reinterpret_cast<get_message_ts_func>(
          dlsym(introspection_type_support_lib, ts_func_name.c_str()));

  if (nullptr == introspection_type_support_handle_func) {
    RCUTILS_LOG_ERROR_NAMED(
        "dynmsg", "failed to load C++ introspection type support function: %s",
        dlerror());
  }

  // Call the function to get the introspection information we want
  const rosidl_message_type_support_t *introspection_ts =
      introspection_type_support_handle_func();
  RCUTILS_LOG_DEBUG_NAMED("dynmsg", "Loaded C++ type support %s",
                          introspection_ts->typesupport_identifier);

  const TypeInfo_Cpp *type_info =
      reinterpret_cast<const TypeInfo_Cpp *>(introspection_ts->data);

  rcl_allocator_t *msg_alloc =
      &msg.get()->get_rcl_serialized_message().allocator;
  uint8_t *data = static_cast<uint8_t *>(
      msg_alloc->allocate(type_info->size_of_, msg_alloc->state));
  type_info->init_function(data,
                           rosidl_runtime_cpp::MessageInitialization::ZERO);
  RosMessage_Cpp ros_msg;
  rclcpp::SerializationBase serializer(introspection_ts);
  auto ptr = msg.get();
  RCLCPP_INFO(this->get_logger(), "%p\n ", (void *)ptr);
  void *des_msg = operator new(type_info->size_of_);

  if ((void *)introspection_ts->data == nullptr)
    RCLCPP_INFO(this->get_logger(), "data is null");
  if ((void *)introspection_ts->func == nullptr)
    RCLCPP_INFO(this->get_logger(), "func is null");
  if ((void *)introspection_ts->typesupport_identifier == nullptr)
    RCLCPP_INFO(this->get_logger(), "type_identifier is null");

  auto t = introspection_ts->func(
      introspection_ts,
      rosidl_typesupport_fastrtps_cpp::typesupport_identifier);
  if (t == nullptr)
    RCLCPP_INFO(this->get_logger(), "t is null");

  RCLCPP_INFO(this->get_logger(), "before deserializer");
  serializer.deserialize_message(ptr, des_msg);
  RCLCPP_INFO(this->get_logger(), "after deserializer");

  ros_msg.data = data;
  ros_msg.type_info = type_info;

  auto yaml_msg = dynmsg::cpp::message_to_yaml(ros_msg);
  auto string_msg = dynmsg::yaml_to_string(yaml_msg, true, false);
  RCLCPP_INFO(this->get_logger(),
              "[generic_subscriber_callback]\nTopic: %s\nDetected Type: "
              "%s\nROS2 Message:\n%s",
              topic_name_.c_str(), detected_type_.c_str(), string_msg.c_str());
}