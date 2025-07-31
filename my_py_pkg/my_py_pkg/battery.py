#!/usr/bin/env python3
from time import sleep

import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed
from functools import partial
 
 
class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery_node")
        self.battery_state = 'full'
        self.run_sim = True
        self.battery_client_ = self.create_client(SetLed, "set_led")
        self.get_logger().info("Battery node has been started")

    def call_set_led(self, led_state: list):
        while not self.battery_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for LED panel node service...")

        assert len(led_state) == 3, "LED state must be a list of 3 boolean values"
        assert all(isinstance(x, bool) for x in led_state), "LED state must be a list of boolean values"

        request = SetLed.Request()
        request.led_array = led_state

        future = self.battery_client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_set_led, request=request))

    def callback_call_set_led(self, future, request):
        response = future.result()
        if response.success:
            self.run_sim = True
        else:
            self.run_sim = False
        self.get_logger().info("LED state: %s, success: %s" % (request.led_array, response.success))

    def run(self):
        while self.run_sim:
            ##### Full -> Empty #####
            sleep(4)
            self.battery_state = 'empty'
            led_command = [False, False, True]
            self.call_set_led(led_command)
            ##### Empty -> Full #####
            sleep(6)
            self.battery_state = 'full'
            led_command = [False, False, False]
            self.call_set_led(led_command)        

 
def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    node.run()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()