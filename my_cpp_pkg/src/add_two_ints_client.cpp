#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
 
class AddTwoIntsClientNode : public rclcpp::Node
{
public:
    AddTwoIntsClientNode() : Node("add_two_ints_client")
    {
        client_ = this-> create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    }

    void callAddTwoInts(int a, int b)
    {
        while (!client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server ...");
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;
        
        // // In case of using 1 argument on callback
        // client_->async_send_request(request, std::bind(&AddTwoIntsClientNode::callbackCallAddTwoInts, this, _1));
        
        // In case of using 2 arguments on callback
        client_->async_send_request(request,
            [this, request](rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future) {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "%d + %d = %d", (int)request->a, (int)request->b, (int)response->sum);
            }
        );
        
    }
 
private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;

    void callbackCallAddTwoInts(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Sum: %d", (int)response->sum);
    }
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClientNode>();
    node->callAddTwoInts(10, 5);
    node->callAddTwoInts(12, 1);
    node->callAddTwoInts(7, 8);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}