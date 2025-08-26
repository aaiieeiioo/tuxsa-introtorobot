import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from airobot_interfaces.action import StringCommand

class BringmeActionClient(Node):
    def __init__(self):  # Constructor
        super().__init__('bringme_action_client')
        # Initialize the action client
        self._action_client = ActionClient(self, StringCommand, 'command')

    def send_goal(self, order):  # Send a goal        
        goal_msg = StringCommand.Goal()  # Create a goal message
        goal_msg.command = order   # send goal (order) : for example "apple"     
        self._action_client.wait_for_server()  # Wait until the server is ready
        # Send the goal asynchronously and process feedback and results
        return self._action_client.send_goal_async(  
            goal_msg, feedback_callback=self.feedback_callback
        )

    def feedback_callback(self, feedback_msg):  # Receive feedback and show progress
        self.get_logger().info(f'Receiving feedback: {feedback_msg.feedback.process}[s] remaining')


def main():
    rclpy.init()
    bringme_action_client = BringmeActionClient()
    order = input('What should I bring?: ')
    
    # Send the goal and get a Future object
    future = bringme_action_client.send_goal(order)  
    # Wait until the goal is sent
    rclpy.spin_until_future_complete(bringme_action_client, future)      
    goal_handle = future.result()  # Get the goal handle

    if not goal_handle.accepted:
        bringme_action_client.get_logger().info('Goal was rejected')
    else:
        bringme_action_client.get_logger().info('Goal was accepted')        
        result_future = goal_handle.get_result_async()  # Get the result asynchronously
        # Wait until the result is ready
        rclpy.spin_until_future_complete(bringme_action_client, result_future)        
        result = result_future.result().result  # Get the result        
        bringme_action_client.get_logger().info(f'Goal result: {result.answer}')  
    
    bringme_action_client.destroy_node()
    rclpy.shutdown()