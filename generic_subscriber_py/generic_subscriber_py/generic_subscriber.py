import time

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


class GenericSubscriber(Node):
    def __init__(self, topic_name):
        super().__init__("generic_subscriber")

        self.topic_name = topic_name
        self.detected_type_str = None
        self.msg_class = None
        self.subscription = None

        self.waitForTopic()
        # Timer to check for publisher status every 2 seconds
        self.create_timer(2.0, self.checkTopicStatus)

    def waitForTopic(self):
        while rclpy.ok():
            topics = dict(self.get_topic_names_and_types())
            if self.count_publishers(self.topic_name) == 0:
                self.get_logger().warn(
                    f"[waitForTopic] Topic '{self.topic_name}' not found. Waiting..."
                )
                time.sleep(0.5)
                continue
            self.detected_type_str = topics[self.topic_name][0]
            self.get_logger().info(
                f"[waitForTopic] Detected type: {self.detected_type_str}"
            )

            self.msg_class = get_message(self.detected_type_str)
            self.createSubscription()
            break

    def checkTopicStatus(self):
        if self.count_publishers(self.topic_name) > 0:
            return

        self.get_logger().warn(
            f"[checkTopicStatus] No publishers on '{self.topic_name}', resetting subscription."
        )
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None
        self.subscription = None
        self.detected_type_str = None
        self.msg_class = None
        self.waitForTopic()

    def createSubscription(self):
        self.subscription = self.create_subscription(
            msg_type=self.msg_class,
            topic=self.topic_name,
            callback=self.genericSubscriberCallback,
            qos_profile=10,
            raw=True,
        )
        self.get_logger().info(
            f"[createSubscription] Subscribed to {self.topic_name} (type: {self.detected_type_str})"
        )

    def genericSubscriberCallback(self, serialized_msg):
        msg = deserialize_message(serialized_msg, self.msg_class)
        self.get_logger().info(f"[genericSubscriberCallback] Received: {msg}")
