#include "rclcpp/rclcpp.hpp"

#include <string>
#include <yaml-cpp/yaml.h>

#include "dynmsg/message_reading.hpp"
#include "dynmsg/typesupport.hpp"
#include "dynmsg/yaml_utils.hpp"

using namespace std::placeholders;

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
  void checkTopicStatus();

  /**
   * @brief
   *
   */
  void waitForTopic();

  /**
   * @brief Create a subscription object
   *
   */
  void createSubscription();

  /**
   * @brief
   *
   * @param msg
   */
  void
  genericSubscriberCallback(std::shared_ptr<rclcpp::SerializedMessage> msg);

  /**
   * @brief Get the Topic Type From String object
   *
   * @param topic_type_str "std_msgs/msg/Bool"
   * @return InterfaceTypeName {"std_msgs", "Bool"}
   */
  void getTopicTypeFromString(InterfaceTypeName &topic_type_pair);
  std::string topic_name_ = "/generic_topic";
  std::string detected_type_;
  rclcpp::GenericSubscription::SharedPtr generic_subscriber_;
  rclcpp::TimerBase::SharedPtr type_check_timer_;
};
