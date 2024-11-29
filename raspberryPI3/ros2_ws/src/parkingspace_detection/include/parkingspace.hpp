#ifndef PARKINGSPACE_HPP
#define PARKINGSPACE_HPP

#include <chrono>
#include <string>
#include <stdint.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "LidarScan.hpp"

class ParkingSpace : public rclcpp::Node
{
public:
	ParkingSpace();

private:
	
	LidarScan scan;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
	rclcpp::Clock::SharedPtr clock_;  // ROS 2 clock
    rclcpp::Time event_1_time_;      // Time for Event 1
    rclcpp::Time event_2_time_;      // Time for Event 2

	void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

#endif // PARKINGSPACE_HPP
