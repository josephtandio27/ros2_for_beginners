#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
#include "my_robot_interfaces/msg/led_panel_state.hpp"

using namespace std::placeholders;
 
class LedPanelNode : public rclcpp::Node
{
public:
    LedPanelNode() : Node("led_panel_node"), led_state_{false, false, false}
    {
        this->declare_parameter("led_states", std::vector<bool>({false, false, false}));
        std::vector<bool> init_led_states = this->get_parameter("led_states").as_bool_array();

        if (init_led_states.size() == 3) {
            std::copy(init_led_states.begin(), init_led_states.end(), led_state_.begin());
        } else {
            RCLCPP_WARN(this->get_logger(), "Parameter 'led_states' has incorrect size. Using default all false.");
            led_state_.fill(false);
        }

        std::ostringstream oss;
        oss << "[";
        std::copy(init_led_states.begin(), init_led_states.end(),
                  std::ostream_iterator<bool>(oss, ", "));
        // Remove the trailing ", " if the vector is not empty
        std::string s = oss.str();
        if (s.length() > 2) { // Checks if "[, " or more is present
            s = s.substr(0, s.length() - 2); // Remove last ", "
        }
        s += "]";
        RCLCPP_INFO_STREAM(this->get_logger(), "Initial LED panel state from parameter: " << s);

        pub_ = this->create_publisher<my_robot_interfaces::msg::LedPanelState>
            ("led_panel_state", 10);
        server_ = this->create_service<my_robot_interfaces::srv::SetLed>(
            "set_led", std::bind(&LedPanelNode::callbackSetLed, this, _1, _2));
        RCLCPP_INFO_STREAM(this->get_logger(), "LED panel node has been started");
    }
 
private:
    std::array<bool, 3> led_state_;
    rclcpp::Publisher<my_robot_interfaces::msg::LedPanelState>::SharedPtr pub_;
    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr server_;


    void callbackSetLed(const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
                        const my_robot_interfaces::srv::SetLed::Response::SharedPtr response)
    {
        led_state_ = request->led_array;
        response->success = true;
        auto msg = my_robot_interfaces::msg::LedPanelState();
        msg.led_array = led_state_;
        pub_->publish(msg);
    }
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}