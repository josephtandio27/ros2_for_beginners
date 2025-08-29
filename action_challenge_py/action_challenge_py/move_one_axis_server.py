#!/usr/bin/env python3
import time
import threading
from math import ceil

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from my_robot_interfaces.action import MoveOneAxis
 
 
class MoveOneAxisServerNode(Node):
    def __init__(self):
        super().__init__("move_one_axis_server")
        self.current_position_ = 50
        self.goal_handle_ : ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.move_server_ = ActionServer(
            self,
            MoveOneAxis,
            "move_one_axis",
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info("Move One Axis Server has been started.")

    def goal_callback(self, goal_request: MoveOneAxis.Goal):
        self.get_logger().info("Goal request received")

        # Check the goal request
        if goal_request.target_position < 0 or goal_request.target_position > 100:
            self.get_logger().info("Goal request rejected. Target position must be between 0 and 100")
            return GoalResponse.REJECT
        if goal_request.velocity < 0:
            self.get_logger().info("Goal request rejected. Velocity must be positive")
            return GoalResponse.REJECT

        # Policy: preempt existing goal when receiving a new goal
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().info("Abort current goal and accept new goal")
                self.goal_handle_.abort()

        self.get_logger().info("Goal request accepted")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        goal_handle.execute()

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Goal request cancelled")
        return CancelResponse.ACCEPT # or REJECT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        feedback = MoveOneAxis.Feedback()
        result = MoveOneAxis.Result()

        # Get request from goal
        target_pos = goal_handle.request.target_position
        target_vel = goal_handle.request.velocity

        # Execute the action
        self.get_logger().info("Executing the goal")
        num_move = ceil(abs(target_pos-self.current_position_)/target_vel)
        for i in range(num_move):
            if not goal_handle.is_active:
                result.final_position = self.current_position_
                result.message = "Goal has been preempted"
                return result
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Canceling the goal")
                goal_handle.canceled()
                result.final_position = self.current_position_
                result.message = "Goal has been canceled"
                return result

            if abs(target_pos-self.current_position_) < target_vel:
                self.current_position_ += target_pos-self.current_position_
            else:
                if target_pos > self.current_position_:
                    self.current_position_ += target_vel
                else:
                    self.current_position_ -= target_vel
            feedback.current_position = self.current_position_
            goal_handle.publish_feedback(feedback)
            self.get_logger().info("Current position: %d" %self.current_position_)
            time.sleep(1)

        # Once done, set goal final state
        goal_handle.succeed()

        # and send the result
        result.final_position = self.current_position_
        result.message = "Goal has been reached"
        return result
 
 
def main(args=None):
    rclpy.init(args=args)
    node = MoveOneAxisServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()