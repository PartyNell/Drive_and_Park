#ifndef PARKINGSPACE_HPP
#define PARKINGSPACE_HPP

#include <chrono>
#include <string>
#include <stdint.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "LidarScan.hpp"
#include "variable.hpp"
#include "std_msgs/msg/int8.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include <math.h>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <ctime>

#define PULSE_FOR_A_REVOLUTION 36
#define WHEEL_DIAMETER 20.0
#define PARKING_SPACE_LIMIT_PARALLEL_LENGTH 95
#define PARKING_SPACE_LIMIT_PARALLEL_DEPTH 70
#define PARKING_SPACE_LIMIT_STRAIGHT_LENGTH 70
#define PARKING_SPACE_LIMIT_STRAIGHT_DEPTH 95

class ParkingSpace : public rclcpp::Node
{
public:
	ParkingSpace();

private:
	std::mutex scan_mutex_;
	std::mutex length_mutex_;

	LidarScan scan;
	float m_length, m_depth;
	ParkingType detected_parking_type_; 

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscription_;
	rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr motors_feedback_subscription_;
	rclcpp::TimerBase::SharedPtr timer_;
	
	void detect_parking_space(const sensor_msgs::msg::LaserScan::SharedPtr msg);
	void increment_parking_space_length(const interfaces::msg::MotorsFeedback::SharedPtr msg);
	void parking_place_info();
};

#endif // PARKINGSPACE_HPP
