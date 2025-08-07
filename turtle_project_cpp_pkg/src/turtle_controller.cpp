#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "my_robot_interfaces/msg/turtle_loc_array.hpp"
#include "my_robot_interfaces/msg/turtle_loc.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;
 
class TurtleControllerNode : public rclcpp::Node
{
public:
    TurtleControllerNode() : Node("turtle_controller")
    {
        target_sub_ = this->create_subscription<my_robot_interfaces::msg::TurtleLocArray>(
            "target_turtles", 10, std::bind(&TurtleControllerNode::callbackTargetTurtles, this, _1)
        );
        pose1_sub_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10,
            std::bind(&TurtleControllerNode::callbackTurtle1, this, _1));
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        
        RCLCPP_INFO(this->get_logger(), "Turtle controller node started");
    }
 
private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose1_sub_;
    rclcpp::Subscription<my_robot_interfaces::msg::TurtleLocArray>::SharedPtr target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

    std::mutex turtle1_pos_mutex_;
    std::array<float, 2> turtle1_pos;
    std::array<float, 2> target_pos;

    void callbackTurtle1(const turtlesim::msg::Pose::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(turtle1_pos_mutex_);
        turtle1_pos = {msg->x, msg->y};

        if (!target_pos.empty()) {
            geometry_msgs::msg::Twist twist;
            twist.linear.x = std::max(std::hypot(target_pos[0] - turtle1_pos[0], target_pos[1] - turtle1_pos[1]), (float)2.0);

            float targetAngle = std::atan2(target_pos[1] - turtle1_pos[1], target_pos[0] - turtle1_pos[0]);
            float angle;
            if (targetAngle * msg->theta >= 0) {
                angle = targetAngle - msg->theta;
            } else {
                angle = targetAngle - msg->theta;
                if (angle > std::numbers::pi) {
                    angle = angle - 2*std::numbers::pi;
                } else if (angle < -std::numbers::pi) {
                    angle = angle + 2*std::numbers::pi;
                }
            }
            twist.angular.z = angle*2;
            twist_pub_->publish(twist);
        }
    }

    void callbackTargetTurtles(const my_robot_interfaces::msg::TurtleLocArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(turtle1_pos_mutex_);
        if (msg->turtles.empty()) {
            RCLCPP_WARN(this->get_logger(), "No target turtles found");
            return;
        }
        
        auto nearest_it = std::min_element(msg->turtles.begin(), msg->turtles.end(),
        [this](const my_robot_interfaces::msg::TurtleLoc& a, const my_robot_interfaces::msg::TurtleLoc& b) {
            return std::hypot(a.x - turtle1_pos[0], a.y - turtle1_pos[1]) < std::hypot(b.x - turtle1_pos[0], b.y - turtle1_pos[1]);
        });

        target_pos = {nearest_it->x, nearest_it->y};
    }
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}