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

        subscription_joystick = this->create_subscription<interfaces::msg::JoystickOrder>("joystick_order", 10, std::bind(&car_command::carCommand, this, _1));
    }

private:
    void carCommand_JoystickOrder(const interfaces::msg::JoystickOrder & joystickOrder)
    {
        //if there is no obstacle detected then the message send is the one sending by the joystick
        if(!obstacle_detection)
        {
            //Joystick order is sent
            publisher_car_control->publish(joystickOrder);
        } 
        else
        {
            //The previous order from obstacle_detection is sent
            publisher_car_control->publish(safety_order);
        }


    }

    void carCommand()
    {
        //set the detect obstacle variable depending on the data receved from the obstacle_detection
    }

    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr publisher_car_control;
    bool obstacle_detection = false;
    interfaces::msg::JoystickOrder safety_order = {
        //start
        //mode
        //throttle
        //steer
        //reverse
    }

    //we should create 2 topics 1.(start and mode) and 2.(throttle, steer and reverse)
    //1. Joystick send it message to car_control (we do not need to process those elements in car_command)
    //2. Joystick send it message to car_command
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<car_command>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}