#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LedPanelState
from my_robot_interfaces.srv import SetLed
 
 
class LedPanelNode(Node):
    def __init__(self):
        super().__init__("led_panel_node")
        self.declare_parameter("led_states", [False, False, False])
        self.led_panel_state = self.get_parameter("led_states").value
        self.get_logger().info("LED panel state: " + str(self.led_panel_state))

        self.led_pub_ = self.create_publisher(LedPanelState, "led_panel_state", 10)
        self.led_server_ = self.create_service(SetLed, "set_led", self.callback_set_led)
        self.get_logger().info("LED panel node has been started")

    def callback_set_led(self, request: SetLed.Request, response: SetLed.Response):
        self.led_panel_state = request.led_array
        response.success = True
        msg = LedPanelState()
        msg.led_array = self.led_panel_state
        self.led_pub_.publish(msg)
        return response
        

 
 
def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()