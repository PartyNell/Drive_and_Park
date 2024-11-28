#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/joystick_order.hpp"

using namespace std::chrono_literals;

class init_autonomous : public rclcpp::Node
{
public:
    init_autonomous()
    : Node("init_autonomous_node"), count_(0)
    {
        publisher_car_order_ = this->create_publisher<interfaces::msg::JoystickOrder>("autonomous_angle_order", 10);
        publisher_init_ok_ = this->create_publisher<interfaces::msg::EventInfo>("init_finished", 10);

        subscription_start_init_ = this->create_subscription<interfaces::msg::EventInfo>("start_init", 10, std::bind(&init_autonomous::start_init_order, this, _1));

        timer_ = this->create_wall_timer(20ms, std::bind(&angle_control::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "init_autonomous_node READY");
    }

private:
    void timer_callback()
    {
        auto car_order = interfaces::msg::JoystickOrder();
        car_order.start = true;
        car_order.mode = 1;
        car_order.throttle = throttle_order;
        car_order.steer = steer_order;
        car_order.reverse = false;
        publisher_car_order_->publish(car_order);
    }

    // Private variables

    float steer_order; // angle sent to the node `car_control_node`
    float throttle_order; // throttle sent to the node `car_control_node`
    bool init_is_finished = false;


    // Publisher
    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr publisher_car_order_;
    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr publisher_car_order_;

    //Subscribers
    rclcpp::Subscription<interfaces::msg::EventInfo>::SharedPtr subscription_start_init_;

    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<init_autonomous>());
    rclcpp::shutdown();
    return 0;
}