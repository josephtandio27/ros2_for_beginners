#!/usr/bin/env python3
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from turtlesim.srv import Kill

import numpy as np 
 
class TurtleManagerNode(Node):
    def __init__(self):
        super().__init__("turtle_manager")
        self.declare_parameter("spawn_rate", 2.0)
        self.spawn_rate_ = self.get_parameter("spawn_rate").value
        self.add_post_set_parameters_callback(self.parameters_callback)

        self.spawned_turtles_ = []
        self.turtle_pose_subs_ = self.create_subscription(Pose, "turtle1/pose", self.callback_turtle_pose, 10)
        self.spawner_client = self.create_client(Spawn, "spawn")
        self.killer_client = self.create_client(Kill, "kill")
        self.get_logger().info("Turtle manager has been started")

    def parameters_callback(self, params: list[Parameter]):
        for param in params:
            if param.name == "spawn_rate":
                self.spawn_rate_ = param.value
                self.get_logger().info("Spawn rate has been changed to: " + str(self.spawn_rate_))
                self.spawn_timer_.cancel()
                self.start_spawning()
    
    ### Listening to turtle 1 pose to kill other turtle ###
    def callback_turtle_pose(self, msg: Pose):
        turtle_indices = [i for i, t in enumerate(self.spawned_turtles_) if 
            np.isclose(msg.x, t["x"], atol=0.5) and np.isclose(msg.y, t["y"], atol=0.5)]
        for i in turtle_indices:
            t = self.spawned_turtles_.pop(i)
            self.kill_turtle(t["name"])

    ### Killing turtles ###
    def kill_turtle(self, name: str):
        while not self.killer_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Killer server ...")

        request = Kill.Request()
        request.name = name

        future = self.killer_client.call_async(request)
        future.add_done_callback(partial(self.callback_kill_turtle, request=request))

    def callback_kill_turtle(self, future, request):
        try:
            response = future.result()
            # self.get_logger().info("Turtle %s killed" % request.name)
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    ### Spawning turtles ###
    def start_spawning(self):
        n = 0
        def encapsulated_spawn_turtle():
            nonlocal n
            x = np.random.random()*10
            y = np.random.random()*10
            theta = np.random.random()*2*np.pi
            name = "turtle_%d" % (n+2)
            self.spawn_turtle(x, y, theta, name)
            n += 1
        self.spawn_timer_ = self.create_timer(self.spawn_rate_, encapsulated_spawn_turtle)

    def spawn_turtle(self, x: float, y: float, theta: float, name: str):
        while not self.spawner_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Spawner server ...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        future = self.spawner_client.call_async(request)
        future.add_done_callback(partial(self.callback_spawn_turtle, request=request))

    def callback_spawn_turtle(self, future, request):
        response = future.result()
        # self.get_logger().info("Turtle %s spawned at (%.2f, %.2f, %.2f)" % (response.name, request.x, request.y, request.theta))
        added_turtle = {"name": response.name, "x": request.x, "y": request.y, "theta": request.theta}
        self.spawned_turtles_.append(added_turtle)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = TurtleManagerNode()
    node.start_spawning()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()