#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/joystick_order.hpp"

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

        RCLCPP_INFO(this->get_logger(), "car_command_node READY");
    }

private:
    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr publisher_car_control_;
    rclcpp::Subscription<interfaces::msg::JoystickOrder>::SharedPtr subscription_joystick_;
    float speed_limit = 0.5;

    void carCommand_JoystickOrder(const interfaces::msg::JoystickOrder & joystickOrder)
    {
        //if there is no obstacle detected then the message send is the one sending by the joystick
        auto control_order = interfaces::msg::JoystickOrder();
        control_order.start = joystickOrder.start;
        control_order.mode = joystickOrder.mode;
        control_order.throttle = joystickOrder.throttle*speed_limit;
        control_order.steer = joystickOrder.steer;
        control_order.reverse = joystickOrder.reverse;

        RCLCPP_INFO(this->get_logger(), "Mode : %d", joystickOrder.mode);
        RCLCPP_INFO(this->get_logger(), "Throttle : %f", control_order.throttle);
        RCLCPP_INFO(this->get_logger(), "Steer : %f", joystickOrder.steer);

        publisher_car_control_->publish(control_order);
    }

    //void carCommand_SafetyOrder(const interfaces::msg::JoystickOrder & safetyOrder)
    //{
    //    //set the safety_order variable depending on the data receved from the obstacle_detection
    //    speed_limit = safetyOrder.speed_coeff;
    //}
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<car_command>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}