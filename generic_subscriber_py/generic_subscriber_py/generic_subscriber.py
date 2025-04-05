import time

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


class GenericSubscriber(Node):
    def __init__(self, topic_name):
        super().__init__("generic_subscriber")

        self.topic_name_ = topic_name
        self.detected_type_str_ = None
        self.msg_class_ = None
        self.subscription_ = None

        self.waitForTopic()
        # Timer to check for publisher status every 2 seconds
        self.create_timer(2.0, self.checkTopicStatus)

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

    def genericSubscriberCallback(self, serialized_msg):
        msg = deserialize_message(serialized_msg, self.msg_class_)
        self.get_logger().info(f"[genericSubscriberCallback] Received: {msg}")
