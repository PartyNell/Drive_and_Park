#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/joystick_order.hpp"
#include "interfaces/msg/speed_info.hpp"

using namespace std;
using placeholders::_1;

class car_command : public rclcpp::Node
{
public:
    car_command()
    : Node("car_command_node")
    {
        publisher_car_control_= this->create_publisher<interfaces::msg::JoystickOrder>("car_command", 10);

        subscription_joystick_ = this->create_subscription<interfaces::msg::JoystickOrder>("joystick_order", 10, std::bind(&car_command::carCommand_JoystickOrder, this, _1));
        subscription_safety_ = this->create_subscription<interfaces::msg::SpeedInfo>("speed_info", 10, std::bind(&car_command::carCommand_SafetyOrder, this, _1));

        RCLCPP_INFO(this->get_logger(), "car_command_node READY");
    }

private:
    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr publisher_car_control_;
    rclcpp::Subscription<interfaces::msg::JoystickOrder>::SharedPtr subscription_joystick_;
    rclcpp::Subscription<interfaces::msg::SpeedInfo>::SharedPtr subscription_safety_;
    float speed_limit = 1.0;

    interfaces::msg::JoystickOrder joystick_order_saved = interfaces::msg::JoystickOrder();

    void carCommand_JoystickOrder(const interfaces::msg::JoystickOrder & joystickOrder)
    {
        //if there is no obstacle detected then the message send is the one sending by the joystick
        joystick_order_saved.start = joystickOrder.start;
        joystick_order_saved.mode = joystickOrder.mode;
        joystick_order_saved.throttle = joystickOrder.throttle;
        joystick_order_saved.steer = joystickOrder.steer;
        joystick_order_saved.reverse = joystickOrder.reverse;

        auto control_order = interfaces::msg::JoystickOrder();
        control_order = joystick_order_saved;
        control_order.throttle *= speed_limit;

        RCLCPP_INFO(this->get_logger(), "Speed coeff : %f", speed_limit);
        RCLCPP_INFO(this->get_logger(), "Mode : %d", joystickOrder.mode);
        RCLCPP_INFO(this->get_logger(), "Throttle : %f", control_order.throttle);
        RCLCPP_INFO(this->get_logger(), "Steer : %f", joystickOrder.steer);

        publisher_car_control_->publish(control_order);
    }

    void carCommand_SafetyOrder(const interfaces::msg::SpeedInfo & safetyOrder)
    {
       //set the safety_order variable depending on the data receved from the obstacle_detection
       speed_limit = safetyOrder.speed_coeff;

       carCommand_JoystickOrder(joystick_order_saved);
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