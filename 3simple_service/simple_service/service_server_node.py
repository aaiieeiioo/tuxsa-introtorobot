import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger   # built-in service type

class SimpleService(Node):
    def __init__(self):
        super().__init__('service_server_node')
        self.srv = self.create_service(Trigger, 'echo', self.callback)

    def callback(self, request, response):
        # Respond with a robotic message
        response.success = True
        response.message = "I got it"
        self.get_logger().info("Service call received. Responded with: I got it")
        return response

def main():
    rclpy.init()
    node = SimpleService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Ctrl+C detected. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()