#ifndef PARKINGSPACE_HPP
#define PARKINGSPACE_HPP

#include <chrono>
#include <string>
#include <stdint.h>
#include <math.h>
#include <cmath>
#include <iostream>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "LidarScan.hpp"
#include "variable.hpp"
#include "std_msgs/msg/bool.hpp"

#define PULSE_FOR_A_REVOLUTION 36
#define WHEEL_DIAMETER 20.0
#define PARKING_SPACE_LIMIT_PARALLEL_LENGTH 190
#define PARKING_SPACE_LIMIT_PARALLEL_DEPTH 95
#define PARKING_SPACE_LIMIT_STRAIGHT_LENGTH 140
#define PARKING_SPACE_LIMIT_STRAIGHT_DEPTH 145

class ParkingSpace : public rclcpp::Node
{
public:
	ParkingSpace();

private:
	std::mutex scan_mutex_;
	std::mutex length_mutex_;

	bool search_in_progress;

	LidarScan scan;
	float m_length, m_depth;
	ParkingType detected_parking_type_; 

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscription_;
	rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr motors_feedback_subscription_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_search_parking;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr type_place_info;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finish_initialization;
	rclcpp::TimerBase::SharedPtr timer_;
	std::shared_ptr<rclcpp::Clock> clock_;
	
	void init_search(const std_msgs::msg::Bool::SharedPtr msg);
	void detect_parking_space(const sensor_msgs::msg::LaserScan::SharedPtr msg);
	void increment_parking_space_length(const interfaces::msg::MotorsFeedback::SharedPtr msg);
	void parking_place_info();

};

#endif // PARKINGSPACE_HPP
