#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

 
class TurtlesimManager : public rclcpp::Node
{
public:
    TurtlesimManager() : Node("turtlesim_manager")
    {
        this->declare_parameter<std::string>("turtle_name");
        turtle_name_ = this->get_parameter("turtle_name").as_string();
        spawner_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        killer_client_ = this->create_client<turtlesim::srv::Kill>("kill");
    }

    void spawnTurtle(float x, float y, float theta)
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

    void killTurtle(std::string name)
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
 
private:
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_client_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr killer_client_;
    std::string turtle_name_;


    void callbackSpawnTurtle(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future) 
    {
        auto response = future.get();
        turtle_name_ = response->name;
        RCLCPP_INFO(this->get_logger(), "%s spawned", turtle_name_.c_str());
    }


};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtlesimManager>();
    node->killTurtle("turtle1");
    std::this_thread::sleep_for(1s);
    node->spawnTurtle(2.5, 2.5, 0);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}