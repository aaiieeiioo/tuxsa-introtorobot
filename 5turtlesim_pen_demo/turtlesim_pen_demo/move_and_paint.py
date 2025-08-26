#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen

class MoveAndPaint(Node):
    def __init__(self):
        super().__init__('move_and_paint')
        
        # Publisher to move the turtle
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Service client to set the pen
        self.client = self.create_client(SetPen, '/turtle1/set_pen')
        self.get_logger().info('Waiting for /turtle1/set_pen service...')
        self.client.wait_for_service()

        # Parameters (so you can override from CLI if you want)
        self.declare_parameter('r', 0)
        self.declare_parameter('g', 255)
        self.declare_parameter('b', 0)
        self.declare_parameter('width', 3)
        self.declare_parameter('off', 0)

        # Set pen once at startup
        self.set_pen_once()

        # Start moving in a gentle arc
        self.timer = self.create_timer(0.05, self.move_step)

    def set_pen_once(self):
        req = SetPen.Request()
        req.r = self.get_parameter('r').get_parameter_value().integer_value
        req.g = self.get_parameter('g').get_parameter_value().integer_value
        req.b = self.get_parameter('b').get_parameter_value().integer_value
        req.width = self.get_parameter('width').get_parameter_value().integer_value
        req.off = self.get_parameter('off').get_parameter_value().integer_value

        future = self.client.call_async(req)

        def _done(_):
            if future.result() is not None:
                self.get_logger().info('Pen set successfully.')
            else:
                self.get_logger().error(f'Failed to set pen: {future.exception()}')

        future.add_done_callback(_done)

    def move_step(self):
        # Constant forward velocity with a slight rotation
        msg = Twist()
        msg.linear.x = 1.5
        msg.angular.z = 1.0
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = MoveAndPaint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
