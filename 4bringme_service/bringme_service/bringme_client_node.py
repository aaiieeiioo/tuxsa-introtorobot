import rclpy
from rclpy.node import Node
from airobot_interfaces.srv import StringCommand


class BringmeClient(Node):
    def __init__(self):  # Constructor
        super().__init__('bringme_client_node')
        self.client = self.create_client(StringCommand, 'command')  # Create the client
        # Wait until the service becomes available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')        
        self.request = StringCommand.Request()  # Create a request instance

    def send_request(self, order):  # Method to send the goal
        self.request.command = order  # Assign order for example "apple"   
        self.future = self.client.call_async(self.request)  # Send async service request


def main():
    rclpy.init()
    bringme_client = BringmeClient()
    order = input('What should I bring?: ')
    bringme_client.send_request(order)

    while rclpy.ok():
        rclpy.spin_once(bringme_client)  # Spin the node once to process callbacks
        if bringme_client.future.done():  # Check if service processing is finished
            try:
                response = bringme_client.future.result()  # Get the service result                  
            except Exception as e:
                bringme_client.get_logger().info(f'Service call failed: {e}')
            else:                
                bringme_client.get_logger().info(  # Display the result
                    f'\nRequest: {bringme_client.request.command} -> Response: {response.answer}')
                break
    bringme_client.destroy_node()  
    rclpy.shutdown()