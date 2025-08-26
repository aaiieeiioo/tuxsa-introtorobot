import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HappySubscriber(Node):
    """A simple subscriber that prints received String messages."""

    def __init__(self):
        super().__init__('happy_subscriber_node')
        # Create subscriber
        self.sub = self.create_subscription(
            String,
            'topic',
            self.callback,
            10
        )

    def callback(self, msg):
        self.get_logger().info(f'Subscribed: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = HappySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Ctrl+C pressed.')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
