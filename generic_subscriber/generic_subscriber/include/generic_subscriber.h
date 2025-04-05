#include "rclcpp/rclcpp.hpp"

#include <string>
#include <yaml-cpp/yaml.h>

#include "dynmsg/message_reading.hpp"
#include "dynmsg/typesupport.hpp"
#include "dynmsg/yaml_utils.hpp"

class GenericSubscriber : public rclcpp::Node {
public:
  /**
   * @brief Construct a new Generic Subscriber object
   *
   */
  GenericSubscriber();

private:
  /**
   * @brief
   *
   */
  void check_topic_status();

  /**
   * @brief
   *
   */
  void wait_for_topic();

  /**
   * @brief Create a subscription object
   *
   */
  void create_subscription();

  /**
   * @brief
   *
   * @param msg
   */
  void
  generic_subscriber_callback(std::shared_ptr<rclcpp::SerializedMessage> msg);

  /**
   * @brief Get the Topic Type From String object
   *
   * @param topic_type_str "std_msgs/msg/Bool"
   * @return std::pair<std::string, std::string> {"std_msgs", "Bool"}
   */
  void
  getTopicTypeFromString(std::pair<std::string, std::string> &topic_type_pair);

  std::string topic_name_ = "/generic_topic";
  std::string detected_type_;
  rclcpp::GenericSubscription::SharedPtr generic_subscriber_;
  rclcpp::TimerBase::SharedPtr type_check_timer_;
};
