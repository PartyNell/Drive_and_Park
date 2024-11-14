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
        publisher_car_control= this->create_publisher<interfaces::msg::JoystickOrder>("car_command", 10);

        subscription_joystick = this->create_subscription<interfaces::msg::JoystickOrder>("joystick_order", 10, std::bind(&car_command::carCommand_JoystickOrder, this, _1));
    }

private:
    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr publisher_car_control;
    rclcpp::Subscription<interfaces::msg::JoystickOrder>::SharedPtr subscription_joystick;
    float speed_limit = 1.0;

    void carCommand_JoystickOrder(const interfaces::msg::JoystickOrder & joystickOrder)
    {
        //if there is no obstacle detected then the message send is the one sending by the joystick
        interfaces::msg::JoystickOrder control_order = {
            joystickOrder.start,
            joystickOrder.mode,
            joystickOrder.throttle*speed_limit,
            joystickOrder.steer,
            joystickOrder.reverse
        }

            publisher_car_control->publish(control_order);
    }

    void carCommand_SafetyOrder(const interfaces::msg::JoystickOrder & safetyOrder)
    {
        //set the safety_order variable depending on the data receved from the obstacle_detection
        speed_limit = safetyOrder.speed_limit;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<car_command>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}