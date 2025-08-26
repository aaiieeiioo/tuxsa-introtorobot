import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HappyPublisher(Node):
    """Publishes a simple countdown, then elapsed time."""

    def __init__(self):
        super().__init__('happy_publisher_node')
        self.pub = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 10  # start value

    def timer_callback(self):
        msg = String()
        if self.i > 0:
            msg.data = f'Happy countdown {self.i}'
        else:
            # No special "action" at zero—just switch to elapsed time
            msg.data = f'Elapsed time {-self.i}'
        self.pub.publish(msg)
        self.get_logger().info(f'Publish: {msg.data}')
        self.i -= 1


def main(args=None):
    rclpy.init(args=args)
    node = HappyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Ctrl+C pressed.')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
