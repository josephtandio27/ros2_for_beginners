#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "my_robot_interfaces/action/move_turtle.hpp"
#include "geometry_msgs/msg/twist.hpp"

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::
    LifecycleNodeInterface::CallbackReturn;
using MoveTurtle = my_robot_interfaces::action::MoveTurtle;
using MoveTurtleGoalHandle = rclcpp_action::ServerGoalHandle<MoveTurtle>;
using namespace std::placeholders;
using namespace std::chrono_literals;
 
namespace final_project {

class TurtlesimManager : public rclcpp_lifecycle::LifecycleNode
{
public:
    TurtlesimManager(const rclcpp::NodeOptions &options);
    
    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &state);
    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &state);
    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);
 
private:
    std::string turtle_name_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::CallbackGroup::SharedPtr cb_group2_;

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_client_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr killer_client_;
    bool turtle1_spawned_;
    
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose1_sub_;
    bool receive_pose_;
    std::array<float, 3> turtle_pose_;
    std::condition_variable cv_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

    rclcpp_action::Server<MoveTurtle>::SharedPtr move_turtle_server_;
    std::mutex mutex_;
    std::shared_ptr<MoveTurtleGoalHandle> goal_handle_;
    int loop_freq_;
    bool is_active_;

    
    void spawnTurtle(float x, float y, float theta);
    void callbackSpawnTurtle(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future);

    void killTurtle(std::string name);

    // Listen to turtle pose
    void callbackPose1(const turtlesim::msg::Pose::SharedPtr msg);

    // Publish twist to turtle
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