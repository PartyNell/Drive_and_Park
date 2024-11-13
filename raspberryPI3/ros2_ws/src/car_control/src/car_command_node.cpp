#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "interfaces/msg/joystick_order.hpp"

using namespace std;

class car_command : public rclcpp::Node
{
public:
    car_command()
    : Node("car_command_node")
    {
        publisher_joystick_order_= this->create_publisher<interfaces::msg::JoystickOrder>("joystick_order", 10);
    }

private:
    void carCommandCallback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

        publisher_joystick_order_->publish(joystickOrderMsg); //Send order to the car_control_node
    }

    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr publisher_joystick_order_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<joystick_to_cmd>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}