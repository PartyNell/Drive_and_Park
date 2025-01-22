#ifndef OBSTACLE_DETECTION_HPP
#define OBSTACLE_DETECTION_HPP

#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/ultrasonic.hpp"
#include "interfaces/msg/speed_info.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <vector>
#include <utility>
#include <iomanip>

#define THRESHOLD_STOP 35
#define THRESHOLD_SLOW 45
#define THRESHOLD_FIRST_SLOW 70
#define THRESHOLD_CAREFUL 100

//////////////////////////////
#define THRESHOLD_PARK_STOP 15
#define LASER_THRESHOLD_BACK_STOP 30
#define LASER_THRESHOLD_FRONT_STOP 120
//////////////////////////////

using namespace std::chrono_literals;

class ObstacleDetection : public rclcpp::Node
{
public:
    ObstacleDetection();
//////////////////////////////////////////////////////    
    bool get_parkmod() const { return parkmod_; }
    void set_parkmod(bool value) { parkmod_ = value; }
    bool get_is_margin_reach_back() const { return is_margin_reach_back_; }
    void set_is_margin_reach_back(bool value) { is_margin_reach_back_ = value; }
    bool get_is_margin_reach_front() const { return is_margin_reach_front_; }
    void set_is_margin_reach_front(bool value) { is_margin_reach_front_ = value; }
//////////////////////////////////////////////////////

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interfaces::msg::SpeedInfo>::SharedPtr publisher_;
    rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_lidar_;
    size_t count_;

    bool will_send_speed_;
    float speed_value_front, speed_value_back;

    ////////////////////////////////////////////////////////
    bool parkmod_;
    bool is_margin_reach_back_;
    bool is_margin_reach_front_;
    ////////////////////////////////////////////////////////

    class SpeedCoefficient {
    public:
        static constexpr float STOP = 0.0;
        static constexpr float WALKING_PACE = 0.15;
        static constexpr float HALF_SPEED = 0.4;
        static constexpr float NORMAL_DETECTION = 0.99;
        static constexpr float NORMAL_NON_DETECTION = 1.0;
    };

    void timer_callback();
    void update_speed_info(bool is_front, int16_t sensor_value);
    void topic_callback(const interfaces::msg::Ultrasonic::SharedPtr msg);
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

#endif // OBSTACLE_DETECTION_HPP