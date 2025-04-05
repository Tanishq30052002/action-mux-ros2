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

        self.wait_for_topic()

        # Timer to check for publisher status every 2 seconds
        self.create_timer(2.0, self.check_topic_status)

    def wait_for_topic(self):
        while rclpy.ok():
            topics = dict(self.get_topic_names_and_types())
            if self.topic_name not in topics or not topics[self.topic_name]:
                self.get_logger().warn(
                    f"Topic '{self.topic_name}' not found. Waiting..."
                )
                time.sleep(0.5)
                continue
            self.detected_type_str = topics[self.topic_name][0]
            self.get_logger().info(
                f"[wait_for_topic] Detected type: {self.detected_type_str}"
            )

            self.msg_class = get_message(self.detected_type_str)
            self.create_subscription_to_topic()
            break

    def check_topic_status(self):
        if self.count_publishers(self.topic_name) > 0:
            return

        self.get_logger().warn(
            f"[check_topic_status] No publishers on '{self.topic_name}', resetting subscription."
        )
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None
        self.subscription = None
        self.detected_type_str = None
        self.msg_class = None
        self.wait_for_topic()

    def create_subscription_to_topic(self):
        self.subscription = self.create_subscription(
            msg_type=self.msg_class,
            topic=self.topic_name,
            callback=self.generic_callback,
            qos_profile=10,
            raw=True,
        )
        self.get_logger().info(
            f"[create_subscription] Subscribed to {self.topic_name} (type: {self.detected_type_str})"
        )

    def generic_callback(self, serialized_msg):
        msg = deserialize_message(serialized_msg, self.msg_class)
        self.get_logger().info(f"[generic_callback] Received: {msg}")
