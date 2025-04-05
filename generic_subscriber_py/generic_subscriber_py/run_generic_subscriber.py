import rclpy

from generic_subscriber_py.generic_subscriber import GenericSubscriber


def main():
    rclpy.init()
    node = GenericSubscriber("/generic_topic")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
