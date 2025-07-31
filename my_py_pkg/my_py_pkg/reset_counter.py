#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool


def main(args=None):
    rclpy.init(args=args)
    node = Node("reset_counter")

    client = node.create_client(SetBool, "reset_counter")
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().warn("Waiting for number_counter server ...")

    request = SetBool.Request()
    request.data = True

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    response = future.result()
    node.get_logger().info("\nCounter reset: %s\nmessage: %s" % (response.success, response.message))
    
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()