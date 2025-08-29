#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/move_one_axis.hpp"
#include "std_msgs/msg/empty.hpp"

using MoveOneAxis = my_robot_interfaces::action::MoveOneAxis;
using MoveOneAxisGoalHandle = rclcpp_action::ClientGoalHandle<MoveOneAxis>;

using namespace std::chrono_literals;
using namespace std::placeholders;

 
class MoveOneAxisClientNode : public rclcpp::Node
{
public:
    MoveOneAxisClientNode() : Node("move_one_axis_client")
    {
        move_one_axis_client_ = rclcpp_action::create_client<MoveOneAxis>(this, "move_one_axis");
        cancel_goal_subscriber_ = this->create_subscription<std_msgs::msg::Empty>(
            "cancel_move", 10, std::bind(&MoveOneAxisClientNode::cancel_goal_callback, this, _1)
        );
    }

    void send_goal(int target_position, int velocity)
    {
        // Wait for the Action server
        move_one_axis_client_->wait_for_action_server(10s);

        // Create a goal
        auto goal = MoveOneAxis::Goal();
        goal.target_position = target_position;
        goal.velocity = velocity;

        // Add callbacks
        auto options = rclcpp_action::Client<MoveOneAxis>::SendGoalOptions();
        options.result_callback = std::bind(&MoveOneAxisClientNode::goal_result_callback, 
            this, _1);
        options.goal_response_callback = std::bind(&MoveOneAxisClientNode::goal_response_callback, 
            this, _1);
        options.feedback_callback = std::bind(&MoveOneAxisClientNode::goal_feedback_callback, 
            this, _1, _2);

        // Send the goal
        RCLCPP_INFO(this->get_logger(), "Sending a goal");
        move_one_axis_client_->async_send_goal(goal, options);
    }
private:
    rclcpp_action::Client<MoveOneAxis>::SharedPtr move_one_axis_client_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_goal_subscriber_;
    MoveOneAxisGoalHandle::SharedPtr goal_handle_;

    // Callback to receive the result once the goal is done
    void goal_result_callback(const MoveOneAxisGoalHandle::WrappedResult &result) {
        auto status = result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Succeeded");
        } else if (status == rclcpp_action::ResultCode::ABORTED) {
            RCLCPP_ERROR(this->get_logger(), "Aborted");
        } else if (status == rclcpp_action::ResultCode::CANCELED) {
            RCLCPP_WARN(this->get_logger(), "Canceled");
        }
        int final_pos = result.result->final_position;
        std::string message = result.result->message;
        RCLCPP_INFO(this->get_logger(), "Final position: %d", final_pos);
        RCLCPP_INFO(this->get_logger(), "Message: %s", message.c_str());
    }

    // Callback to know if the goal was accepted or rejected
    void goal_response_callback(const MoveOneAxisGoalHandle::SharedPtr &goal_handle) {
        if (!goal_handle) {
            RCLCPP_WARN(this->get_logger(), "Goal was rejected by the server");
            return;
        }
        this->goal_handle_ = goal_handle;
        RCLCPP_INFO(this->get_logger(), "Goal was accepted by the server");
    }

    // Callback to receive feedback during goal execution
    void goal_feedback_callback(const MoveOneAxisGoalHandle::SharedPtr &goal_handle,
        const std::shared_ptr<const MoveOneAxis::Feedback> feedback) {
            (void)goal_handle;
            int current_pos = feedback->current_position;
            RCLCPP_INFO(this->get_logger(), "Current position: %d", current_pos);
        }

    void cancel_goal_callback(const std_msgs::msg::Empty::SharedPtr msg) {
        (void)msg;
        RCLCPP_INFO(this->get_logger(), "Canceling the goal");
        move_one_axis_client_->async_cancel_goal(goal_handle_);
    }
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveOneAxisClientNode>();
    node->send_goal(10, 5);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}