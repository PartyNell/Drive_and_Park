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
        subscription_autonomous_ = this->create_subscription<interfaces::msg::JoystickOrder>("autonomous_car_order", 10, std::bind(&car_command::carCommand_AutonomousOrder, this, _1));

        RCLCPP_INFO(this->get_logger(), "car_command_node READY");
    }

private:
    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr publisher_car_control_;
    rclcpp::Subscription<interfaces::msg::JoystickOrder>::SharedPtr subscription_joystick_;
    rclcpp::Subscription<interfaces::msg::SpeedInfo>::SharedPtr subscription_safety_;
    rclcpp::Subscription<interfaces::msg::JoystickOrder>::SharedPtr subscription_autonomous_;

    float speed_limit_front = 1.0;
	float speed_limit_back = 1.0;

    interfaces::msg::JoystickOrder order_saved = interfaces::msg::JoystickOrder();

    void carCommand_JoystickOrder(const interfaces::msg::JoystickOrder & joystick_order)
    {
        //Update START and MODE
        order_saved.start = joystick_order.start;
        order_saved.mode = joystick_order.mode;

        //Publish joystick order if the mode is 0 or 2
        if(order_saved.mode == 0 || order_saved.mode == 2)
        {
            publishOrder(joystick_order.throttle, joystick_order.steer, joystick_order.reverse);
        }
    }

    void carCommand_AutonomousOrder(const interfaces::msg::JoystickOrder & autonomousOrder)
    {
        //Publish autonomous order if the mode is 1
        if(order_saved.mode == 1)
        {
            publishOrder(autonomousOrder.throttle, autonomousOrder.steer, autonomousOrder.reverse);
        }
    }

    void carCommand_SafetyOrder(const interfaces::msg::SpeedInfo & safetyOrder)
    {
      	//set the safety_order variable depending on the data receved from the obstacle_detection
       	speed_limit_front = safetyOrder.speed_coeff_front;
		speed_limit_back = safetyOrder.speed_coeff_back;

      	carCommand_JoystickOrder(order_saved);
    }

    void publishOrder(float throttle, float steer, bool reverse)
    {
        //Save data without modification from detect obstacle
        order_saved.throttle = throttle;
        order_saved.steer = steer;
        order_saved.reverse = reverse;

        //Apply speed coefficient from detect_obstacle
        auto control_order = interfaces::msg::JoystickOrder();
        control_order = order_saved;

		if (control_order.reverse) {
			control_order.throttle *= speed_limit_back;
			RCLCPP_INFO(this->get_logger(), "Backward -> Speed coeff: %f", speed_limit_back);
		}
		else {
			control_order.throttle *= speed_limit_front;
			RCLCPP_INFO(this->get_logger(), "Forward -> Speed coeff: %f", speed_limit_front);
		}

        RCLCPP_INFO(this->get_logger(), "Mode : %d", control_order.mode);
        RCLCPP_INFO(this->get_logger(), "Throttle : %f", control_order.throttle);
        RCLCPP_INFO(this->get_logger(), "Steer : %f", control_order.steer);

        publisher_car_control_->publish(control_order);
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