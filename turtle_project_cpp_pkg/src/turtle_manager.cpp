#include <numbers>
#include <algorithm>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/msg/pose.hpp"
#include "my_robot_interfaces/msg/turtle_loc_array.hpp"
#include "my_robot_interfaces/msg/turtle_loc.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;
 
class TurtleManagerNode : public rclcpp::Node
{
public:
    TurtleManagerNode() : Node("turtle_manager"), n_turtles(2)
    {
        this->declare_parameter("spawn_rate", 2.0);
        spawn_rate_ = static_cast<float>(this->get_parameter("spawn_rate").as_double());

        param_callback_handle_ = this->add_post_set_parameters_callback(
            std::bind(&TurtleManagerNode::parametersCallback, this, _1)
        );

        spawner_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        killer_client_ = this->create_client<turtlesim::srv::Kill>("kill");
        pose1_sub_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10,
            std::bind(&TurtleManagerNode::callbackTurtle1Pose, this, _1));
        target_pub_ = this->create_publisher<my_robot_interfaces::msg::TurtleLocArray>("target_turtles", 10);
        spawn_timer_ = this->create_wall_timer(std::chrono::duration<double>(spawn_rate_), std::bind(&TurtleManagerNode::spawnTurtle, this));

        RCLCPP_INFO(this->get_logger(), "Turtle manager node started");
    }
 
private:
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_client_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr killer_client_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose1_sub_;
    rclcpp::Publisher<my_robot_interfaces::msg::TurtleLocArray>::SharedPtr target_pub_;
    rclcpp::TimerBase::SharedPtr spawn_timer_;
    PostSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    std::array<float, 2> turtle1_pos;
    int n_turtles;
    float spawn_rate_;
    
    std::mutex available_turtles_mutex_;
    my_robot_interfaces::msg::TurtleLocArray available_turtles;

    void parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
    {
        for (const auto &param: parameters) {
            spawn_rate_ = static_cast<float>(param.as_double());
            spawn_timer_->cancel();
            spawn_timer_->reset();
            spawn_timer_ = this->create_wall_timer(std::chrono::duration<double>(spawn_rate_), std::bind(&TurtleManagerNode::spawnTurtle, this));
        }
    }

    void callbackTurtle1Pose(const turtlesim::msg::Pose::SharedPtr msg)
    {
        turtle1_pos = {msg->x, msg->y};

        std::string turtle_to_kill;

        {
            std::lock_guard<std::mutex> lock(available_turtles_mutex_);
            auto it = std::find_if(available_turtles.turtles.begin(),
                available_turtles.turtles.end(),
                [this](const my_robot_interfaces::msg::TurtleLoc& turtle) {
                    return std::abs(turtle.x - turtle1_pos[0]) <= 0.5 && std::abs(turtle.y - turtle1_pos[1]) <= 0.5;
                }
            );

            if (it != available_turtles.turtles.end()) {
                turtle_to_kill = it->name;
            }
        }

        if (!turtle_to_kill.empty()) {
            killTurtle(turtle_to_kill);
        }
    }

    void spawnTurtle()
    {
        while (!spawner_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the Turtlesim Spawn server ...");
        }

        std::mt19937_64 rng(std::random_device{}()); // Mersenne Twister engine
        std::uniform_real_distribution<double> dist(0.0, 1.0);

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = dist(rng) * 10;
        request->y = dist(rng) * 10;
        request->theta = (dist(rng) * 2 - 1) *std::numbers::pi;
        request->name = "turtle" + std::to_string(n_turtles);

        n_turtles++;

        spawner_client_->async_send_request(request,
            [this, request](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future) {
                auto response = future.get();
                auto turtle = my_robot_interfaces::msg::TurtleLoc();
                turtle.name = response->name;
                turtle.x = request->x;
                turtle.y = request->y;
                available_turtles.turtles.push_back(turtle);
                target_pub_->publish(available_turtles);
            }
        );
    }

    void killTurtle(const std::string& name)
    {
        while (!killer_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the Turtlesim Kill server ...");
        }

        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = name;

        killer_client_->async_send_request(request,
            [this, request](rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future) {
                auto response = future.get();
            }
        );

        removeTurtleByName(request->name);
        target_pub_->publish(available_turtles);
    }

    void removeTurtleByName(const std::string& name_to_remove)
    {
        std::lock_guard<std::mutex> lock(available_turtles_mutex_);
        auto it = std::find_if(available_turtles.turtles.begin(), available_turtles.turtles.end(),
        [&](const my_robot_interfaces::msg::TurtleLoc& turtle) {
            return turtle.name == name_to_remove;
        });

        if (it != available_turtles.turtles.end()) {
            available_turtles.turtles.erase(it);
        } else {
            RCLCPP_WARN(this->get_logger(), "Turtle not found: %s", name_to_remove.c_str());
        }
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