#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/joystick_order.hpp"

using namespace std::chrono_literals;

class Navigation : public rclcpp::Node
{
  public:
    Navigation()
    : Node("car_navigation"), count_(0)
    {
      publisher_car_order_ = this->create_publisher<interfaces::msg::JoystickOrder>("autonomous_car_order", 10);

      timer_ = this->create_wall_timer(50ms, std::bind(&Navigation::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto car_order = interfaces::msg::JoystickOrder();
      car_order.start = true;
      car_order.mode = 1;
      car_order.throttle = 0.6;
      car_order.steer = 0.0;
      car_order.reverse = false;

      publisher_->publish(car_order);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr publisher_car_order_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}