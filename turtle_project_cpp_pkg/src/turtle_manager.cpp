#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/msg/pose.hpp"

using namespace std::placeholders;
 
class TurtleManagerNode : public rclcpp::Node
{
public:
    TurtleManagerNode() : Node("turtle_manager")
    {
        spawner_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        killer_client_ = this->create_client<turtlesim::srv::Kill>("kill");
        pose1_sub_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10,
            std::bind(&TurtleManagerNode::callback_turtle1_pose, this, _1));
    }
 
private:
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_client_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr killer_client_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose1_sub_;

    std::vector<float> turtle1_pos;

    void callback_turtle1_pose(const turtlesim::msg::Pose::SharedPtr msg)
    {
        turtle1_pos = {msg->x, msg->y};
    }
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}