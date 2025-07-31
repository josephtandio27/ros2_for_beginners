// // Library effective with Windows
// #include <windows.h>

// Library effective with Linux
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
 
class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery_node"), battery_state_("full"), run_sim(true)
    {
        client_ = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");
    }

    void run() {
        while (run_sim) {
            // Battery full -> empty
            sleep(4);
            battery_state_ = "empty";
            callSetLed(std::array<bool, 3>{false, false, true});
            RCLCPP_INFO_STREAM(this->get_logger(), "Battery state: " << battery_state_);
            // Battery empty -> full
            sleep(6);
            battery_state_ = "full";
            callSetLed(std::array<bool, 3>{false, false, false});
            RCLCPP_INFO_STREAM(this->get_logger(), "Battery state: " << battery_state_);
        }
    }
 
private:
    rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedPtr client_;
    std::string battery_state_;
    bool run_sim;

    void callSetLed(const std::array<bool, 3>& led_state)
    {
        while (!client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the led state server ...");
        }

        auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
        request->led_array = led_state;

        client_->async_send_request(request, std::bind(&BatteryNode::callbackCallSetLed, this, _1));
    }

    void callbackCallSetLed(rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedFuture future) {
        auto response = future.get();
        RCLCPP_INFO_STREAM(this->get_logger(), "Success: " << std::boolalpha << response->success);
    }

};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    node->run();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}