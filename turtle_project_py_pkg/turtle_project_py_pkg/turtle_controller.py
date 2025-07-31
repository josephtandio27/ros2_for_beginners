#!/usr/bin/env python3
from functools import partial

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

import numpy as np
from scipy.spatial import KDTree
 
 
class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.turtle_subscribers_ = {}
        self.target_pos_ = {}
        # Store watchdog timer for each spawned turtle
        self.turtle_timers_ = {}
        # Stores last message timestamp for each spawned turtle
        self.last_msg_time_ = {}
        # How often to check for new/disappeared topics
        self.CHECK_TOPIC_INTERVAL = 1.0
        # How long before a silent subscriber is considered inactive
        self.SUBSCRIBER_TIMEOUT = 0.1

        self.check_topic_timer_ = self.create_timer(self.CHECK_TOPIC_INTERVAL, self.callback_manage_topic)
        self.turtle1_pose_sub_ = self.create_subscription(Pose, "/turtle1/pose", self.callback_move_to_target, 10)
        self.move_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.get_logger().info("Turtle controller has been started")
    
    ####################################
    ### Get and remove turtle topics ###
    ####################################
    def callback_manage_topic(self):
        topic_names_and_types = self.get_topic_names_and_types()
        if not topic_names_and_types:
            self.get_logger().warn("No topics found or discovery not complete yet.")

        active_pose_topics = set()
        for topic_name, _ in topic_names_and_types:
            if 'turtle' in topic_name and 'pose' in topic_name and '/turtle1/pose' not in topic_name:
                active_pose_topics.add(topic_name)
                if topic_name not in self.turtle_subscribers_:
                    self._add_turtle_subscription(topic_name)

        # Remove subscriptions for topics that no longer exist in the ROS graph
        topics_to_remove_from_graph = [
            t for t in self.turtle_subscribers_.keys()
            if t not in active_pose_topics
        ]
        for topic_name in topics_to_remove_from_graph:
            self._remove_turtle_subscription(topic_name)

    def _add_turtle_subscription(self, topic_name):
        """Helper to add a new turtle subscription and its associated watchdog."""
        self.turtle_subscribers_[topic_name] = self.create_subscription(Pose,
                                                                        topic_name,
                                                                        partial(self.callback_turtle_pose, topic_name),
                                                                        10)
        self.target_pos_[topic_name] = np.array([0.0, 0.0])
        # Initialize last message time to now, and set up a watchdog timer
        self.last_msg_time_[topic_name] = self.get_clock().now().nanoseconds
        self.turtle_timers_[topic_name] = self.create_timer(
            self.SUBSCRIBER_TIMEOUT,
            partial(self._check_subscriber_activity, topic_name)
        )

    def _check_subscriber_activity(self, topic_name: str):
        """Watchdog callback to check if a subscriber has become inactive."""
        if topic_name not in self.last_msg_time_ or topic_name not in self.turtle_subscribers_:
            # This can happen if _remove_turtle_subscription was called via callback_manage_topic
            # right before this timer's callback got executed. Just return.
            return

        current_time = self.get_clock().now().nanoseconds
        last_time = self.last_msg_time_[topic_name]

        if (current_time - last_time)/10**9 > self.SUBSCRIBER_TIMEOUT:
            self._remove_turtle_subscription(topic_name)
        
    def callback_turtle_pose(self, topic_name: str, msg: Pose):
        self.target_pos_[topic_name] = np.array([msg.x, msg.y])
        # Update the last message time and reset the watchdog timer
        self.last_msg_time_[topic_name] = self.get_clock().now().nanoseconds
        if topic_name in self.turtle_timers_:
            self.turtle_timers_[topic_name].reset()

    def _remove_turtle_subscription(self, topic_name):
        """Helper to remove a turtle subscription and all associated data."""
        if topic_name in self.turtle_subscribers_:
            self.destroy_subscription(self.turtle_subscribers_.pop(topic_name))
        if topic_name in self.target_pos_:
            del self.target_pos_[topic_name]
        if topic_name in self.last_msg_time_:
            del self.last_msg_time_[topic_name]
        if topic_name in self.turtle_timers_:
            self.turtle_timers_[topic_name].cancel() # Always cancel before destroying
            self.destroy_timer(self.turtle_timers_.pop(topic_name))
        

    ################################################################
    ### Get current turtle 1 pose and move toward closest target ###
    ################################################################
    def callback_move_to_target(self, msg: Pose):
        self.turtle1_pose_ = np.array([msg.x, msg.y, msg.theta])
        # Get closest target
        _, closest_topic = self.find_shortest_target(self.turtle1_pose_[:-1])
        if closest_topic:
            # Calculate velocity (Twist)
            vel_msg = self.compute_velocity(self.turtle1_pose_, self.target_pos_[closest_topic])
            # Publish velocity
            self.move_pub_.publish(vel_msg)            

    def find_shortest_target(self, current_pos: np.array):
        if not self.target_pos_:
            return None, None        
        # Prepare data for KD-Tree
        names = list(self.target_pos_.keys())
        points = np.array(list(self.target_pos_.values()))
        # Build the KD-Tree
        tree = KDTree(points)
        # Query the KD-Tree for the nearest neighbor
        distance, idx = tree.query(current_pos)
        # Get the name of the closest point
        closest_topic = names[idx]
        return distance, closest_topic

    def compute_velocity(self, current_pose: np.array, target_pos: np.array):
        displacement = target_pos - current_pose[:-1]
        target_angle = np.arctan2(displacement[1], displacement[0])
        if target_angle * current_pose[-1] >= 0:
            angle = target_angle-current_pose[-1]
        else:
            angle = target_angle-current_pose[-1]
            if angle > np.pi:
                angle = angle - 2*np.pi
            elif angle < -np.pi:
                angle = angle + 2*np.pi

        distance = np.linalg.norm(displacement)
        moving_time = 1.0
        msg = Twist()
        # Add 2 for min speed
        msg.linear.x = max(distance/moving_time, 2.0)
        msg.angular.z = angle/moving_time*2.0
        return msg
        

        
 
def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()