import time, random
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from airobot_interfaces.action import StringCommand  # Import custom action definition


class BringmeActionServer(Node):
    def __init__(self):
        super().__init__('bringme_action_server')
        self._action_server = ActionServer(
            self, StringCommand, 'command', 
            execute_callback=self.execute_callback,
        )
        self.food = ['apple', 'banana', 'candy']

    def execute_callback(self, goal_handle):
        feedback = StringCommand.Feedback()
        count = random.randint(5, 10)

        while count > 0:
            self.get_logger().info(f'Sending feedback: {count}[s] remaining')     
            feedback.process = f'{count}'
            goal_handle.publish_feedback(feedback)  
            count -= 1  
            time.sleep(1)

        item = goal_handle.request.command
        result = StringCommand.Result()
        if item in self.food:
            result.answer = f'Yes, here is your {item}.'
        else:
            result.answer = f'Sorry, I could not find {item}.'
        goal_handle.succeed()
        self.get_logger().info(f'Goal result: {result.answer}')
        return result


def main():
    rclpy.init()
    bringme_action_server = BringmeActionServer()
    print('Server started')
    try:
        rclpy.spin(bringme_action_server)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()
    print('Server shutdown')