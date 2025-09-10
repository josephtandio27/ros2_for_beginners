#include "final_project/diff_drive_gz_manager.hpp"

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::
    LifecycleNodeInterface::CallbackReturn;
using MoveTurtle = my_robot_interfaces::action::MoveTurtle;
using MoveTurtleGoalHandle = rclcpp_action::ServerGoalHandle<MoveTurtle>;
using namespace std::placeholders;
using namespace std::chrono_literals;


namespace final_project {
    
DiffDriveGzManager::DiffDriveGzManager(const rclcpp::NodeOptions &options) : 
    LifecycleNode("move_diff_drive_action_server", options), robot_pose_({0.0, 0.0, 0.0}),
    pred_pose_({0.0, 0.0, 0.0}), loop_freq_(100), is_active_(false)
     
{
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    RCLCPP_INFO(this->get_logger(), "Move differential drive action server in unconfigured state");    
}

LifecycleCallbackReturn DiffDriveGzManager::on_configure(const rclcpp_lifecycle::State &state) 
{
    (void)state;
    RCLCPP_INFO(this->get_logger(), "Move differential drive action server in on_configure state");

    // Declare parameters
    this->declare_parameter<std::string>("vel_topic");
    vel_topic_ = this->get_parameter("vel_topic").as_string();

    // Create buffer and listener timer to get robot pose
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create publisher to send velocity
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/" + vel_topic_, 10);

    // Create action server
    move_turtle_server_ = rclcpp_action::create_server<MoveTurtle>(
        this,
        "move_turtle",
        std::bind(&DiffDriveGzManager::goal_callback, this, _1, _2),
        std::bind(&DiffDriveGzManager::cancel_callback, this, _1),
        std::bind(&DiffDriveGzManager::handle_accepted_callback, this, _1),
        rcl_action_server_get_default_options(),
        cb_group_
    );
    RCLCPP_INFO(this->get_logger(), "Move turtle action server has been started");
    
    return rclcpp_lifecycle::LifecycleNode::on_configure(state);
}

LifecycleCallbackReturn DiffDriveGzManager::on_cleanup(const rclcpp_lifecycle::State &state) 
{
    (void)state;
    RCLCPP_INFO(this->get_logger(), "Move differential drive action server in on_cleanup state");
    this->undeclare_parameter("vel_topic");
    tf_buffer_.reset();
    tf_listener_.reset();
    twist_pub_.reset();
    move_turtle_server_.reset();
    return rclcpp_lifecycle::LifecycleNode::on_cleanup(state);
}

LifecycleCallbackReturn DiffDriveGzManager::on_activate(const rclcpp_lifecycle::State &state)
{
    (void)state;
    RCLCPP_INFO(this->get_logger(), "Move differential drive action server in on_activate state");
    is_active_ = true;
    return rclcpp_lifecycle::LifecycleNode::on_activate(state);
}

LifecycleCallbackReturn DiffDriveGzManager::on_deactivate(const rclcpp_lifecycle::State &state)
{
    (void)state;
    RCLCPP_INFO(this->get_logger(), "Move differential drive action server in on_deactivate state");
    is_active_ = false;
    return rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
}

LifecycleCallbackReturn DiffDriveGzManager::on_shutdown(const rclcpp_lifecycle::State &state)
{
    (void)state;
    RCLCPP_INFO(this->get_logger(), "Move differential drive action server in on_shutdown state");
    this->undeclare_parameter("vel_topic");
    tf_buffer_.reset();
    tf_listener_.reset();
    twist_pub_.reset();
    move_turtle_server_.reset();
    return rclcpp_lifecycle::LifecycleNode::on_shutdown(state);
}

void DiffDriveGzManager::getRobotPose()
{
    std::string to_frame = "odom";
    std::string from_frame = "base_footprint";
    
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
        // Look up the transform from 'odom' to 'base_footprint'
        tf_stamped = tf_buffer_->lookupTransform(
            to_frame,
            from_frame,
            tf2::TimePointZero
        );
        auto q = tf_stamped.transform.rotation;
        double rot_z = tf2::getYaw<geometry_msgs::msg::Quaternion>(q);
        robot_pose_  = {static_cast<float>(tf_stamped.transform.translation.x),
                        static_cast<float>(tf_stamped.transform.translation.y),
                        static_cast<float>(rot_z)};
    }
    catch (const tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", 
            to_frame.c_str(), from_frame.c_str(), ex.what());
    }
}

// Publish twist to robot
void DiffDriveGzManager::publishTwist(const float lin_vel_x, const float ang_vel_z)
{
    // Create a unique pointer to the message to avoid copy operation
    auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
    
    // Populate the message fields
    twist_msg->linear.x = lin_vel_x;
    twist_msg->angular.z = ang_vel_z;
    
    // Publish the unique pointer
    twist_pub_->publish(std::move(twist_msg));
}

rclcpp_action::GoalResponse DiffDriveGzManager::goal_callback(const rclcpp_action::GoalUUID &uuid, 
    std::shared_ptr<const MoveTurtle::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Goal has been received");

    if (!is_active_) {
        RCLCPP_WARN(this->get_logger(), "Move diff drive action server is not active");
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

rclcpp_action::CancelResponse DiffDriveGzManager::cancel_callback(
    const std::shared_ptr<MoveTurtleGoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

void DiffDriveGzManager::handle_accepted_callback(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing the goal");
    execute_goal(goal_handle);
}

void DiffDriveGzManager::execute_goal(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle) {
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
        this->getRobotPose();
        RCLCPP_INFO(this->get_logger(), "Initial pose: x = %f, y = %f, theta = %f", 
            robot_pose_[0], robot_pose_[1], robot_pose_[2]);
        pred_pose_ = predictPos(robot_pose_, linear_vel_x, angular_vel_z, duration);
        RCLCPP_INFO(this->get_logger(), "Predicted pose: x = %f, y = %f, theta = %f", 
            pred_pose_[0], pred_pose_[1], pred_pose_[2]);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    RCLCPP_INFO(this->get_logger(), "Publishing linear x velocity: %f, angular z velocity: %f",
        linear_vel_x, angular_vel_z);
    for (int i = 0; i < num_iterations; i++) {
        if (goal_handle->is_canceling()) {
            std::lock_guard<std::mutex> lock(mutex_);
            this->getRobotPose();
            // Set final state and return result
            result->final_pos_x = robot_pose_[0];
            result->final_pos_y = robot_pose_[1];
            result->final_theta = robot_pose_[2];
            result->pos_error = std::hypot(pred_pose_[0] - robot_pose_[0], pred_pose_[1] - robot_pose_[1]);
            result->success = false;
            goal_handle->canceled(result);
            return;
        }
        publishTwist(linear_vel_x, angular_vel_z);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            this->getRobotPose();
            feedback->current_pos_x = robot_pose_[0];
            feedback->current_pos_y = robot_pose_[1];
            feedback->current_theta = robot_pose_[2];
        }
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
    }
    // Set velocity to zero to stop the robot
    publishTwist(0.0, 0.0);
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        std::lock_guard<std::mutex> lock(mutex_);
        this->getRobotPose();
        RCLCPP_INFO(this->get_logger(), "Final pose: x = %f, y = %f, theta = %f", 
            robot_pose_[0], robot_pose_[1], robot_pose_[2]);
        result->final_pos_x = robot_pose_[0];
        result->final_pos_y = robot_pose_[1];
        result->final_theta = robot_pose_[2];
        result->pos_error = std::hypot(pred_pose_[0] - robot_pose_[0],
            pred_pose_[1] - robot_pose_[1]);
    }
    result->success = true;
    goal_handle->succeed(result);
}

std::array<float, 3> DiffDriveGzManager::predictPos(const std::array<float, 3>& init_pos, const float lin_vel_x, 
    const float ang_vel_z, const float duration)
{
    std::array<float, 3> final_pos = init_pos;
    final_pos[0] += 0.5*lin_vel_x*(sin(final_pos[2]+ang_vel_z*duration)-
        sin(final_pos[2]))/ang_vel_z;
    final_pos[1] += 0.5*lin_vel_x*(-cos(final_pos[2]+ang_vel_z*duration)+
        cos(final_pos[2]))/ang_vel_z;
    final_pos[2] += 0.5*ang_vel_z*duration;
    final_pos[2] = fmod(final_pos[2], 2.0 * M_PI);
    if (final_pos[2] > M_PI) {
        final_pos[2] -= 2.0 * M_PI;
    }
    if (final_pos[2] < -M_PI) {
        final_pos[2] += 2.0 * M_PI;
    }
    return final_pos;
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(final_project::DiffDriveGzManager)