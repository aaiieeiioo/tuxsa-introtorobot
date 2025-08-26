#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class SimpleNavClient(Node):
    """Send a single NavigateToPose goal to Nav2."""

    def __init__(self):
        super().__init__('simple_nav_client')

        # Parameters (override via --ros-args -p x:=... etc.)
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw_deg', 0.0)
        self.declare_parameter('frame_id', 'map')

        self._ac = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._sent = False
        self.timer = self.create_timer(0.2, self._tick)

    def _tick(self):
        if not self._ac.wait_for_server(timeout_sec=0.0):
            self.get_logger().info('Waiting for /navigate_to_pose action server...')
            return
        if self._sent:
            return

        # Read parameters
        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        yaw_deg = float(self.get_parameter('yaw_deg').value)
        frame_id = str(self.get_parameter('frame_id').value)

        yaw = math.radians(yaw_deg)

        # Build goal
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'Sending goal → ({x:.2f}, {y:.2f}, yaw={yaw_deg:.1f}°) in {frame_id}'
        )
        self._sent = True
        self._ac.send_goal_async(goal, feedback_callback=self._on_feedback) \
                .add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by server.')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted.')
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        # fb.distance_remaining, fb.navigation_time, fb.number_of_recoveries, etc.
        try:
            self.get_logger().info(f'Distance remaining: {fb.distance_remaining:.2f} m')
        except Exception:
            pass  # some distros omit this field

    def _on_result(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation finished with result code: {result.result}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = SimpleNavClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
