#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool
 
 
class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.count_ = Int64()
        self.count_.data = 0
        self.server_ = self.create_service(SetBool, "reset_counter", self.callback_reset)
        self.subscriber_ = self.create_subscription(Int64, "number", self.callback_count, 10)
        self.publisher_ = self.create_publisher(Int64, "number_count", 10)
        self.get_logger().info("Number counter has been started")

    def callback_count(self, msg: Int64):
        self.count_.data += msg.data
        self.publisher_.publish(self.count_)

    def callback_reset(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            self.count_.data = 0
            response.success = True
            response.message = "Counter has been reset"
        else:
            response.success = False
            response.message = "Counter has not been reset"
        return response        


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()