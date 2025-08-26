import time
import rclpy
from rclpy.node import Node
from airobot_interfaces.srv import StringCommand

class BringmeService(Node):  # Bringme service class
    def __init__(self):  # Constructor
        super().__init__('bringme_service')
        # Create the service (service type, service name, callback function)
        self.service = self.create_service(StringCommand, 'command', self.callback)
        self.food = ['apple', 'banana', 'candy']   

    def callback(self, request, response):  # Callback function
        time.sleep(5)  # Simulate processing time
        item = request.command
        if item in self.food:
            response.answer = f'Yes, here is your {item}.'
        else:
            response.answer = f'Sorry, I could not find {item}.'
        self.get_logger().info(f'Response: {response.answer}')
        return response


def main():  # main function
    rclpy.init()
    bringme_service = BringmeService()
    try:
        rclpy.spin(bringme_service)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()
    print('Server shutdown')