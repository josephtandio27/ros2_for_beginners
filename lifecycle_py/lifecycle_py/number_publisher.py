#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from example_interfaces.msg import Int64

class NumberPublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__("number_publisher")
        self.number_ = None
        self.publish_frequency_ = None
        self.get_logger().info("In unconfigured state")
        self.number_publisher_ = None
        self.number_timer_ = None

    # Create ROS2 communications, connect to hardware, etc.
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.number_ = 1
        self.publish_frequency_ = 1.0
        # Publisher will not send message until the state is in active state
        self.number_publisher_ = self.create_lifecycle_publisher(Int64, "number", 10)
        self.number_timer_ = self.create_timer(
            1.0 / self.publish_frequency_, self.publish_number)
        self.number_timer_.cancel()
        self.get_logger().info("In on_configure state")
        self.get_logger().info("Number publisher has been started.")
        return super().on_configure(state)

    # Destroy ROS2 communications, disconnect from hardware, etc.
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("In on_cleanup state")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return super().on_cleanup(state)

    # Activate/enable hardware
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("In on_activate state")
        self.number_timer_.reset()
        return super().on_activate(state)

    # Deactivate/disable hardware
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("In on_deactivate state")
        self.number_timer_.cancel()
        return super().on_deactivate(state)

    # Deactivate and cleanup all resources
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("In on_shutdown state")
        if self.number_publisher_ is not None:
            self.destroy_lifecycle_publisher(self.number_publisher_)
        if self.number_timer_ is not None:
            self.destroy_timer(self.number_timer_)
        return super().on_shutdown(state)

    # Error state will be activated in case there is raised exception in the other states
    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("In on_error state")
        return super().on_error(state)

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg)
        self.number_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
