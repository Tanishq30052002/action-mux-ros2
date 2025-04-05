import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


class DynamicDeserializerNode(Node):
    def __init__(self, topic_name, msg_type_str):
        super().__init__("dynamic_deserializer_node")

        # Get the message class from the type string
        self.msg_class = get_message(msg_type_str)
        self.topic_name = topic_name

        # Create a raw subscription so we receive serialized messages
        self.create_subscription(
            msg_type=self.msg_class,
            topic=self.topic_name,
            callback=self.callback,
            qos_profile=10,
            raw=True,  # Important: receive serialized bytes
        )

        self.get_logger().info(
            f"Subscribed to {self.topic_name} with type {msg_type_str}"
        )

    def callback(self, serialized_msg):
        # Deserialize the message
        msg = deserialize_message(serialized_msg, self.msg_class)
        self.get_logger().info(f"Deserialized message: {msg}")


def main():
    rclpy.init()

    # Customize topic and message type
    topic_name = "/generic_topic"
    msg_type_str = "geometry_msgs/msg/Twist"

    node = DynamicDeserializerNode(topic_name, msg_type_str)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
