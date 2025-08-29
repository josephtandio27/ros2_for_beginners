#!/usr/bin/env python3
from concurrent.futures import Future

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from std_msgs.msg import Empty
from my_robot_interfaces.action import MoveOneAxis
 
class MoveOneAxisClientNode(Node):
    def __init__(self):
        super().__init__("move_one_axis_client")
        self.goal_handle_ : ClientGoalHandle = None
        self.move_one_axis_client_ = ActionClient(self, MoveOneAxis, "move_one_axis")
        self.cancel_subscriber_ = self.create_subscription(
            Empty,
            "cancel_move",
            self.cancel_goal,
            10
        )
        self.get_logger().info("Move One Axis Client has been started.")
        self.get_logger().info("Cancel move via ros2 topic /cancel_move")

    def send_goal(self, target_position, velocity):
        # Wait for the server
        self.move_one_axis_client_.wait_for_server(timeout_sec=10.0)

        # Create a goal
        goal = MoveOneAxis.Goal()
        goal.target_position = target_position
        goal.velocity = velocity

        # Send the goal
        self.get_logger().info("Sending goal")
        self.move_one_axis_client_.send_goal_async(
            goal, feedback_callback=self.goal_feedback_callback
        ).add_done_callback(self.goal_response_callback)

    def cancel_goal(self, msg: Empty):
        self.get_logger().info("Send a cancel request")
        if self.goal_handle_ is not None:
            self.goal_handle_.cancel_goal_async()

    def goal_feedback_callback(self, feedback_msg):
        current_pos = feedback_msg.feedback.current_position
        self.get_logger().info("Current position: %d" %current_pos)

    def goal_response_callback(self, future: Future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal was accepted by server")
            self.goal_handle_.get_result_async().add_done_callback(
                self.goal_result_callback
            )
        else:
            self.get_logger().error("Goal was rejected by server")

    def goal_result_callback(self, future: Future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info("Final position: %d" % result.final_position)
        self.get_logger().info(result.message)

 
def main(args=None):
    rclpy.init(args=args)
    node = MoveOneAxisClientNode()
    node.send_goal(100, 2)
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()