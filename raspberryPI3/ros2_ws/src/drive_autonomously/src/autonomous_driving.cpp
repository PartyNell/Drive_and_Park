#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/joystick_order.hpp"

using namespace std::chrono_literals;

class AutonomousDriving : public rclcpp::Node
{
  public:
    AutonomousDriving()
    : Node("autonomous_driving"), count_(0)
    {
      publisher_car_order_ = this->create_publisher<interfaces::msg::JoystickOrder>("autonomous_car_order", 10);

      timer_ = this->create_wall_timer(50ms, std::bind(&AutonomousDriving::timer_callback, this));

      RCLCPP_INFO(this->get_logger(), "autonomous_driving READY");
    }

  private:
    void timer_callback()
    {
      auto car_order = interfaces::msg::JoystickOrder();
      car_order.start = true;
      car_order.mode = 1;
      car_order.throttle = 0.7;
      car_order.steer = 0.0;
      car_order.reverse = false;

      publisher_car_order_->publish(car_order);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr publisher_car_order_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutonomousDriving>());
  rclcpp::shutdown();
  return 0;
}