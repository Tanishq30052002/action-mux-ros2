import rclpy

from generic_subscriber_py.generic_subscriber import GenericSubscriber


def main():
    rclpy.init()
    node = GenericSubscriber("/generic_topic")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down node gracefully...")
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
