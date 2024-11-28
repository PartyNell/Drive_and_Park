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


	void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

#endif // PARKINGSPACE_HPP
