#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using namespace std::placeholders;
 
class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter")
    {
        counter_.data = 0;
        server_ = this->create_service<example_interfaces::srv::SetBool>(
            "reset_counter", 
            std::bind(&NumberCounterNode::callbackResetCounter, this, _1, _2)
        );
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
            "number", 10, std::bind(&NumberCounterNode::callbackGetNumber, this, _1)
        );
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        RCLCPP_INFO(this->get_logger(), "Number counter has been started");
    }
 
private:
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
    example_interfaces::msg::Int64 counter_;

    void callbackGetNumber(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        counter_.data += msg->data;
        publisher_->publish(counter_);
    }

    void callbackResetCounter(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                            const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if (request->data){
            counter_.data = 0;
            response->success = true;
            response->message = "Counter has been reset";
        }
        else{
            response->success = false;
            response->message = "Counter has not been reset";
        }
    }


};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}