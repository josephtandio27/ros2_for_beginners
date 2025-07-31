#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using namespace std::chrono_literals;

 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("reset_counter");

    auto client = node->create_client<example_interfaces::srv::SetBool>("reset_counter");
    while (!client->wait_for_service(1s))
    {
        RCLCPP_WARN(node->get_logger(), "Waiting for the server ...");
    }

    auto request = std::make_shared<example_interfaces::srv::SetBool::Request>();
    request->data = true;

    auto future = client->async_send_request(request);
    rclcpp::spin_until_future_complete(node, future);
    auto response = future.get();
    RCLCPP_INFO_STREAM(node->get_logger(), "success: " 
        << std::boolalpha << response->success << ", message: " << response->message);  
    
    rclcpp::shutdown();
    return 0;
}