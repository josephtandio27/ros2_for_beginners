#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "final_project/turtlesim_manager.hpp"

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::
    LifecycleNodeInterface::CallbackReturn;
using MoveTurtle = my_robot_interfaces::action::MoveTurtle;
using MoveTurtleGoalHandle = rclcpp_action::ServerGoalHandle<MoveTurtle>;
using namespace std::placeholders;
using namespace std::chrono_literals;
 

namespace final_project {

TurtlesimManager::TurtlesimManager(const rclcpp::NodeOptions &options) : 
    LifecycleNode("move_turtle_action_server", options), pose_received_(false),
    loop_freq_(100), is_active_(false)
{
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    turtle1_spawned_ = true;
    RCLCPP_INFO(this->get_logger(), "Move turtle action server in unconfigured state");
}
    
LifecycleCallbackReturn TurtlesimManager::on_configure(const rclcpp_lifecycle::State &state)
{
    (void)state;
    RCLCPP_INFO(this->get_logger(), "Move turtle action server in on_configure state");
    
    // Declare parameters
    this->declare_parameter<std::string>("turtle_name");
    turtle_name_ = this->get_parameter("turtle_name").as_string();

    // Create clients
    spawner_client_ = this->create_client<turtlesim::srv::Spawn>("spawn",
        rclcpp::SystemDefaultsQoS(), cb_group_);
    killer_client_ = this->create_client<turtlesim::srv::Kill>("kill",
        rclcpp::SystemDefaultsQoS(), cb_group_);
    
    // Create subscriber
    rclcpp::SubscriptionOptions options;
    options.callback_group = cb_group_;
    pose1_sub_ = this->create_subscription<turtlesim::msg::Pose>("/" + turtle_name_ + "/pose", 10,
        std::bind(&TurtlesimManager::callbackPose1, this, _1), options);

    // Create publisher
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/" + turtle_name_ + "/cmd_vel", 
        10);

    // Create action server
    move_turtle_server_ = rclcpp_action::create_server<MoveTurtle>(
        this,
        "move_turtle",
        std::bind(&TurtlesimManager::goal_callback, this, _1, _2),
        std::bind(&TurtlesimManager::cancel_callback, this, _1),
        std::bind(&TurtlesimManager::handle_accepted_callback, this, _1),
        rcl_action_server_get_default_options(),
        cb_group_
    );
    
    if (turtle1_spawned_) {
        RCLCPP_INFO(this->get_logger(), "Killing turtle1");
        this->killTurtle("turtle1");
        std::this_thread::sleep_for(5s);
        turtle1_spawned_ = false;
    }

    RCLCPP_INFO(this->get_logger(), "Spawning %s", turtle_name_.c_str());
    this->spawnTurtle(4, 4, 0);

    RCLCPP_INFO(this->get_logger(), "Move turtle action server has been started");

    return rclcpp_lifecycle::LifecycleNode::on_configure(state);
}

LifecycleCallbackReturn TurtlesimManager::on_cleanup(const rclcpp_lifecycle::State &state)
{
    (void)state;
    RCLCPP_INFO(this->get_logger(), "Move turtle action server in on_cleanup state");
    this->killTurtle(turtle_name_);
    this->undeclare_parameter("turtle_name");
    spawner_client_.reset();
    killer_client_.reset();
    pose1_sub_.reset();
    twist_pub_.reset();
    move_turtle_server_.reset();
    return rclcpp_lifecycle::LifecycleNode::on_cleanup(state);
}

LifecycleCallbackReturn TurtlesimManager::on_activate(const rclcpp_lifecycle::State &state)
{
    (void)state;
    RCLCPP_INFO(this->get_logger(), "Move turtle action server in on_activate state");
    is_active_ = true;
    return rclcpp_lifecycle::LifecycleNode::on_activate(state);
}

LifecycleCallbackReturn TurtlesimManager::on_deactivate(const rclcpp_lifecycle::State &state)
{
    (void)state;
    RCLCPP_INFO(this->get_logger(), "Move turtle action server in on_deactivate state");
    is_active_ = false;
    return rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
}

LifecycleCallbackReturn TurtlesimManager::on_shutdown(const rclcpp_lifecycle::State &state)
{
    (void)state;
    RCLCPP_INFO(this->get_logger(), "Move turtle action server in on_shutdown state");
    this->undeclare_parameter("turtle_name");
    spawner_client_.reset();
    killer_client_.reset();
    pose1_sub_.reset();
    twist_pub_.reset();
    move_turtle_server_.reset();
    return rclcpp_lifecycle::LifecycleNode::on_shutdown(state);
}

void TurtlesimManager::spawnTurtle(float x, float y, float theta)
{
    while (!spawner_client_->wait_for_service(1s)) {
        RCLCPP_WARN(this->get_logger(), "Waiting for the Turtlesim Node ...");
    }

    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->x = x;
    request->y = y;
    request->theta = theta;
    request->name = turtle_name_;

    spawner_client_->async_send_request(request, 
        std::bind(&TurtlesimManager::callbackSpawnTurtle, this, _1));
}

void TurtlesimManager::killTurtle(std::string name)
{
    while (!killer_client_->wait_for_service(1s)) {
        RCLCPP_WARN(this->get_logger(), "Waiting for the Turtlesim Node ...");
    }

    auto request = std::make_shared<turtlesim::srv::Kill::Request>();
    request->name = name;

    killer_client_->async_send_request(request, 
        [this, request](rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "%s killed", request->name.c_str());
        });
}

void TurtlesimManager::callbackSpawnTurtle(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future) 
{
    auto response = future.get();
    turtle_name_ = response->name;
    RCLCPP_INFO(this->get_logger(), "%s spawned", turtle_name_.c_str());
}

// Listen to turtle pose
void TurtlesimManager::callbackPose1(const turtlesim::msg::Pose::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    turtle_pose_ = {msg->x, msg->y, msg->theta};
    pose_received_ = true;
    cv_.notify_one();
}

// Publish twist to turtle
void TurtlesimManager::publishTwist(const float lin_vel_x, const float ang_vel_z)
{
    // Create a unique pointer to the message to avoid copy operation
    auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
    
    // Populate the message fields
    twist_msg->linear.x = lin_vel_x;
    twist_msg->angular.z = ang_vel_z;
    
    // Publish the unique pointer
    twist_pub_->publish(std::move(twist_msg));
}


rclcpp_action::GoalResponse TurtlesimManager::goal_callback(const rclcpp_action::GoalUUID &uuid, 
    std::shared_ptr<const MoveTurtle::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Goal has been received");

    if (!is_active_) {
        RCLCPP_WARN(this->get_logger(), "Move turtle action server is not active");
        return rclcpp_action::GoalResponse::REJECT;
    }

    // Policy: refuse new goal if current goal is active
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (goal_handle_) {
            if (goal_handle_->is_active()) {
                RCLCPP_WARN(this->get_logger(), "Current goal is active. Goal has been rejected");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
    }

    if (std::abs(goal->linear_vel_x) > 5.0) {
        RCLCPP_WARN(this->get_logger(), "Goal rejected: linear velocity must be less than 5 m/s");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (std::abs(goal->angular_vel_z) > M_PI/2) {
        RCLCPP_WARN(this->get_logger(), "Goal rejected: angular velocity must be less than pi/2 rad/s");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "Goal has been accepted");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

rclcpp_action::CancelResponse TurtlesimManager::cancel_callback(
    const std::shared_ptr<MoveTurtleGoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

void TurtlesimManager::handle_accepted_callback(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing the goal");
    execute_goal(goal_handle);
}

void TurtlesimManager::execute_goal(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        this->goal_handle_ = goal_handle;
    }

    // Get request from goal
    float linear_vel_x = goal_handle->get_goal()->linear_vel_x;
    float angular_vel_z = goal_handle->get_goal()->angular_vel_z;
    float duration = goal_handle->get_goal()->duration;

    // Execute the action
    auto result = std::make_shared<MoveTurtle::Result>();
    auto feedback = std::make_shared<MoveTurtle::Feedback>();
    const int num_iterations = static_cast<int>(duration * loop_freq_);
    rclcpp::Rate loop_rate(loop_freq_);
    
    // Wait for first pose message
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return pose_received_; });
        pose_received_ = false;
    }
    std::array<float, 3> pred_pose = predictPos(turtle_pose_, 
        linear_vel_x, angular_vel_z, duration);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    RCLCPP_INFO(this->get_logger(), "Publishing linear x velocity: %f, angular z velocity: %f",
        linear_vel_x, angular_vel_z);
    for (int i = 0; i < num_iterations; i++) {
        if (goal_handle->is_canceling()) {
            std::lock_guard<std::mutex> lock(mutex_);
            // Set final state and return result
            result->final_pos_x = turtle_pose_[0];
            result->final_pos_y = turtle_pose_[1];
            result->final_theta = turtle_pose_[2];
            // TODO
            result->pos_error = std::hypot(pred_pose[0] - turtle_pose_[0], pred_pose[1] - turtle_pose_[1]);
            result->success = false;
            goal_handle->canceled(result);
            return;
        }
        publishTwist(linear_vel_x, angular_vel_z);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            feedback->current_pos_x = turtle_pose_[0];
            feedback->current_pos_y = turtle_pose_[1];
            feedback->current_theta = turtle_pose_[2];
        }
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        result->final_pos_x = turtle_pose_[0];
        result->final_pos_y = turtle_pose_[1];
        result->final_theta = turtle_pose_[2];
        // TODO
        result->pos_error = std::hypot(pred_pose[0] - turtle_pose_[0],
            pred_pose[1] - turtle_pose_[1]);
    }
    result->success = true;
    goal_handle->succeed(result);
}

std::array<float, 3> TurtlesimManager::predictPos(const std::array<float, 3>& init_pos, const float lin_vel_x, 
    const float ang_vel_z, const float duration)
{
    std::array<float, 3> final_pos = init_pos;
    final_pos[0] += lin_vel_x * cos(final_pos[2]) * duration;
    final_pos[1] += lin_vel_x * sin(final_pos[2]) * duration;
    final_pos[2] += ang_vel_z * duration;
    final_pos[2] = fmod(final_pos[2], 2.0 * M_PI);
    if (final_pos[2] > M_PI) {
        final_pos[2] -= 2.0 * M_PI;
    }
    if (final_pos[2] < -M_PI) {
        final_pos[2] += 2.0 * M_PI;
    }
    return final_pos;
}
    
} // namespace final_project

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(final_project::TurtlesimManager)