#ifndef AUTO_PARKING_HPP
#define AUTO_PARKING_HPP

#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/ultrasonic.hpp"
#include "interfaces/msg/speed_info.hpp"


class AutoParking : public rclcpp::Node
{
public:
    AutoParking();

private:
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();

};

#endif // AUTO_PARKING_HPP
