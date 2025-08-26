import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger

class EchoClient(Node):
    def __init__(self):
        super().__init__('service_client_node')
        self.client = self.create_client(Trigger, 'echo')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = Trigger.Request()

    def send_request(self):
        return self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    node = EchoClient()
    future = node.send_request()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            if future.done():
                try:
                    resp = future.result()
                except Exception as e:
                    node.get_logger().error(f'Service call failed: {e}')
                else:
                    node.get_logger().info(f'success={resp.success}, message="{resp.message}"')
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()