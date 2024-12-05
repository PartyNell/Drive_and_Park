#ifndef OBSTACLE_DETECTION_HPP
#define OBSTACLE_DETECTION_HPP

#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/ultrasonic.hpp"
#include "interfaces/msg/speed_info.hpp"

#define THRESHOLD_STOP 35
#define THRESHOLD_SLOW 45
#define THRESHOLD_FIRST_SLOW 70
#define THRESHOLD_CAREFUL 100

using namespace std::chrono_literals;

class ObstacleDetection : public rclcpp::Node
{
public:
    ObstacleDetection();

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interfaces::msg::SpeedInfo>::SharedPtr publisher_;
    rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr subscription_;
    size_t count_;

    bool will_send_speed_;
    float speed_value_front, speed_value_back;

    class SpeedCoefficient {
    public:
        static constexpr float STOP = 0.0;
        static constexpr float WALKING_PACE = 0.15;
        static constexpr float HALF_SPEED = 0.4;
        static constexpr float NORMAL = 1.0;
    };

    void timer_callback();
    void update_speed_info(bool is_front, int16_t sensor_value);
    void topic_callback(const interfaces::msg::Ultrasonic::SharedPtr msg);
};

#endif // OBSTACLE_DETECTION_HPP
