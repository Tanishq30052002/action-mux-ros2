#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

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

  std::string topic_name_ = "/generic_topic";
  std::string detected_type_;
  rclcpp::GenericSubscription::SharedPtr generic_subscriber_;
  rclcpp::TimerBase::SharedPtr type_check_timer_;
};
