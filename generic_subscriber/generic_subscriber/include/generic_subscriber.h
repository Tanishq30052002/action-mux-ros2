#include "rclcpp/rclcpp.hpp"

#include <string>
#include <yaml-cpp/yaml.h>

#include "dynmsg/message_reading.hpp"
#include "dynmsg/typesupport.hpp"
#include "dynmsg/yaml_utils.hpp"
#include "rclcpp/serialization.hpp"

using namespace std::placeholders;

class GenericSubscriber : public rclcpp::Node {
public:
  /**
   * @brief Construct a new Generic Subscriber object, which waits for a topic.
   * Once the topic is detected, it checks the topic status
   *
   * @param topic_name The name of the topic to subscribe to (e.g.,
   * "/topic_name").
   */
  GenericSubscriber(const std::string &topic_name);

private:
  /**
   * @brief Check the status of the topic. If no active publishers are found, it
   * reset the subscription and waits for the topic again.
   *
   */
  void checkTopicStatus();

  /**
   * @brief Wait for the topic to be published. Once the topic is detected, it
   * creates the subscription to the topic.
   *
   */
  void waitForTopic();

  /**
   * @brief Create a Subscription object to the detected topic type.
   *
   */
  void createSubscription();

  /**
   * @brief Callback function for the generic subscription. It receives the and
   * currently identifies the type of the message. and print the initialized
   * values of the message.
   *
   * @param msg The received message as a serialized message.
   *
   * @note This function is currently not deserializing the message. It is just
   * detecting the type of the msg and prints the initialized values.
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
  std::string topic_name_;
  std::string detected_type_;
  rclcpp::GenericSubscription::SharedPtr generic_subscriber_;
  rclcpp::TimerBase::SharedPtr type_check_timer_;
};
