import time

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


class GenericSubscriber(Node):
    """GenericSubscriber is a ROS2 node that subscribes to a topic
    and dynamically detects its message type at runtime.
    It waits for the topic to be available and creates a subscription
    to the topic once it is detected.

    Args:
        topic_name (str): the name of the topic to subscribe to.
    """

    def __init__(self, topic_name):
        super().__init__("generic_subscriber")

        self.topic_name_ = topic_name
        self.detected_type_str_ = None
        self.msg_class_ = None
        self.subscription_ = None

        self.waitForTopic()
        # Timer to check for publisher status every 2 seconds
        self.create_timer(2.0, self.checkTopicStatus)

    """
    waitForTopic: Waits for the topic to be available and detects its type.
    It creates a subscription to the topic once it is available.
    This is useful for dynamically discovering the topic type at runtime.
    """

    def waitForTopic(self):
        while rclpy.ok():
            topics = dict(self.get_topic_names_and_types())
            if self.count_publishers(self.topic_name_) == 0:
                self.get_logger().warn(
                    f"[waitForTopic] Topic '{self.topic_name_}' not found. Waiting..."
                )
                time.sleep(0.5)
                continue
            self.detected_type_str_ = topics[self.topic_name_][0]
            self.get_logger().info(
                f"[waitForTopic] Detected type: {self.detected_type_str_}"
            )

            self.msg_class_ = get_message(self.detected_type_str_)
            self.createSubscription()
            break

    """
    checkTopicStatus: Checks if there are any publishers on the topic.
    If not, it destroys the current subscription and resets the detected type.
    This is useful for handling cases where the topic may go away or change type.
    It is called periodically by a timer.
    It checks if the topic has any publishers and resets the subscription if not.
    It also logs a warning if the topic is not found.
    """

    def checkTopicStatus(self):
        if self.count_publishers(self.topic_name_) > 0:
            return

        self.get_logger().warn(
            f"[checkTopicStatus] No publishers on '{self.topic_name_}', resetting subscription."
        )
        if self.subscription_ is not None:
            self.destroy_subscription(self.subscription_)
            self.subscription_ = None
        self.subscription_ = None
        self.detected_type_str_ = None
        self.msg_class_ = None
        self.waitForTopic()

    """
    createSubscription: Creates a subscription to the topic with the detected type.
    It uses a callback function to handle incoming messages.
    This is called once the topic type is detected.
    It uses the detected type to create a subscription.
    """

    def createSubscription(self):
        self.subscription_ = self.create_subscription(
            msg_type=self.msg_class_,
            topic=self.topic_name_,
            callback=self.genericSubscriberCallback,
            qos_profile=10,
            raw=True,
        )
        self.get_logger().info(
            f"[createSubscription] Subscribed to {self.topic_name_} (type: {self.detected_type_str_})"
        )

    """
    genericSubscriberCallback: Callback function for the subscription.
    It receives serialized messages and deserializes them.
    It logs the received message.
    This is called when a message is received on the subscription.
    It uses the detected type to deserialize the message.
    It logs the received message.
    """

    def genericSubscriberCallback(self, serialized_msg):
        msg = deserialize_message(serialized_msg, self.msg_class_)
        self.get_logger().info(f"[genericSubscriberCallback] Received: {msg}")
