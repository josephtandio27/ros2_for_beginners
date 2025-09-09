#include <cmath>
#include <mutex>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/move_turtle.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::
    LifecycleNodeInterface::CallbackReturn;
using MoveTurtle = my_robot_interfaces::action::MoveTurtle;
using MoveTurtleGoalHandle = rclcpp_action::ServerGoalHandle<MoveTurtle>;
using namespace std::placeholders;
using namespace std::chrono_literals;
 
namespace final_project {

class DiffDriveGzManager : public rclcpp_lifecycle::LifecycleNode
{
public:
    DiffDriveGzManager(const rclcpp::NodeOptions &options);
    
    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &state);
    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &state);
    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);
 
private:
    std::string vel_topic_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    // For receiving the pose of the robot
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // x, y, theta
    std::array<float, 3> robot_pose_;
    std::array<float, 3> pred_pose_;

    // For publishing the twist to the robot
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

    // For the action server
    rclcpp_action::Server<MoveTurtle>::SharedPtr move_turtle_server_;
    std::mutex mutex_;
    std::shared_ptr<MoveTurtleGoalHandle> goal_handle_;
    int loop_freq_;
    bool is_active_;
    

    // Listen to robot pose
    void getRobotPose();

    // Publish twist to robot
    void publishTwist(const float lin_vel_x, const float ang_vel_z);    

    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid, 
        std::shared_ptr<const MoveTurtle::Goal> goal);

    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<MoveTurtleGoalHandle> goal_handle);

    void handle_accepted_callback(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle);

    void execute_goal(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle);

    std::array<float, 3> predictPos(const std::array<float, 3>& init_pos, const float lin_vel_x, 
        const float ang_vel_z, const float duration);
    
};

} // namespace final_project