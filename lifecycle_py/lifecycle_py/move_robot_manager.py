#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

class MoveRobotManager(Node):
    def __init__(self):
        super().__init__("move_robot_manager")
        self.declare_parameter("managed_node_names", rclpy.Parameter.Type.STRING_ARRAY)
        node_names = self.get_parameter("managed_node_names").value
        self.get_logger().info("Managed node names: " + str(node_names))
        self.client_list_ = []
        for node_name in node_names:
            service_change_state_name = "/" + node_name + "/change_state"
            self.client_list_.append(self.create_client(ChangeState, service_change_state_name))
        
    def change_states(self, transition: Transition):
        for client in self.client_list_:
            client.wait_for_service()
            request = ChangeState.Request()
            request.transition = transition
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            # Check if the state change was successful
            if future.result() is not None and future.result().success:
                self.get_logger().info(client.service_name + " switched to " + transition.label)
            else:
                raise RuntimeError(client.service_name + " failed to switch to " + transition.label)
    
    def initialization_sequence(self):
        # Unconfigured to Inactive
        self.get_logger().info("Trying to switch to configuring")
        transition = Transition()
        transition.id = Transition.TRANSITION_CONFIGURE
        transition.label = "configure"
        try:
            self.change_states(transition)
        except RuntimeError as e:
            self.get_logger().error(str(e))
            transition = Transition()
            transition.id = Transition.TRANSITION_UNCONFIGURED_SHUTDOWN
            transition.label = "shutdown"
            self.change_states(transition)
            return
        self.get_logger().info("Configuring OK, now inactive")

        # sleep just for the example
        time.sleep(3)

        # Inactive to Active
        self.get_logger().info("Trying to switch to activating")
        transition = Transition()
        transition.id = Transition.TRANSITION_ACTIVATE
        transition.label = "activate"
        try:
            self.change_states(transition)
        except RuntimeError as e:
            self.get_logger().error(str(e))
            transition = Transition()
            transition.id = Transition.TRANSITION_UNCONFIGURED_SHUTDOWN
            transition.label = "shutdown"
            self.change_states(transition)
            return
        self.get_logger().info("Activating OK, now active")


def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotManager()
    node.initialization_sequence()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
