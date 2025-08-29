#include <cmath>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/move_one_axis.hpp"

using MoveOneAxis = my_robot_interfaces::action::MoveOneAxis;
using MoveOneAxisGoalHandle = rclcpp_action::ServerGoalHandle<MoveOneAxis>;
using namespace std::placeholders;


class MoveOneAxisServerNode : public rclcpp::Node
{
public:
    MoveOneAxisServerNode() : Node("move_one_axis")
    {
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        move_one_axis_server_ = rclcpp_action::create_server<MoveOneAxis>(
            this,
            "move_one_axis",
            std::bind(&MoveOneAxisServerNode::goal_callback, this, _1, _2),
            std::bind(&MoveOneAxisServerNode::cancel_callback, this, _1),
            std::bind(&MoveOneAxisServerNode::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(),
            cb_group_
        );
        
        RCLCPP_INFO(this->get_logger(), "Move one axis server has been started");
    }
 
private:
    rclcpp_action::Server<MoveOneAxis>::SharedPtr move_one_axis_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp_action::GoalUUID preempted_goal_id_;
    std::mutex mutex_;
    std::shared_ptr<MoveOneAxisGoalHandle> goal_handle_;
    int current_position_{50};

    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid, 
        std::shared_ptr<const MoveOneAxis::Goal> goal) {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Goal has been received");

        // Validate goal
        if ((goal->target_position < 0) || (goal->target_position > 100)) {
            RCLCPP_ERROR(this->get_logger(), "Goal has been rejected");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal->velocity < 0) {
            RCLCPP_ERROR(this->get_logger(), "Goal has been rejected");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        // Policy: preempt existing goal when receiving a new valid goal
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (goal_handle_) {
                if (goal_handle_->is_active()) {
                    RCLCPP_WARN(this->get_logger(), "Aborting current goal and accepting new goal");
                    preempted_goal_id_ = goal_handle_->get_goal_id();
                }
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Goal has been accepted");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<MoveOneAxisGoalHandle> goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Received cancel request");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

    void handle_accepted_callback(const std::shared_ptr<MoveOneAxisGoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing the goal");
        execute_goal(goal_handle);
    }

    void execute_goal(const std::shared_ptr<MoveOneAxisGoalHandle> goal_handle) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            this->goal_handle_ = goal_handle;
        }

        // Get request from goal
        int target_position = goal_handle->get_goal()->target_position;
        int velocity = goal_handle->get_goal()->velocity;

        // Execute the action
        auto result = std::make_shared<MoveOneAxis::Result>();
        auto feedback = std::make_shared<MoveOneAxis::Feedback>();
        rclcpp::Rate loop_rate(1.0);
        int num_move = ceil(abs(target_position-this->current_position_)/velocity);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        for (int i = 0; i < num_move; i++) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (goal_handle->get_goal_id() == preempted_goal_id_) {
                    result->final_position = this->current_position_;
                    result->message = "Goal has been preempted";
                    goal_handle->abort(result);
                    return;
                }
            }
            if (goal_handle->is_canceling()) {
                result->final_position = this->current_position_;
                result->message = "Goal has been canceled";
                goal_handle->canceled(result);
                return;
            }
            if (abs(target_position-this->current_position_) < velocity) {
                this->current_position_ += target_position-this->current_position_;
            } else if (target_position > this->current_position_) {
                this->current_position_ += velocity;
            } else {
                this->current_position_ -= velocity;
            }
            feedback->current_position = this->current_position_;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Current position: %d", this->current_position_);
            loop_rate.sleep();
        }

        // Set final state and return result
        result->final_position = this->current_position_;
        result->message = "Goal has been reached";
        goal_handle->succeed(result);
    }
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveOneAxisServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}