#ifndef PARKINGSPACE_HPP
#define PARKINGSPACE_HPP

#include <chrono>
#include <string>
#include <stdint.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/LaserScan"
#include "LidarScan.hpp"
#define NB_POINTS 1024


ParkingSpace : public rclcpp::Node
{
  public:
    ParkingSpace();
    ~ParkingSpace();

  private:
    LidarScan scan;


    void topic_callback(const std_msgs::msg::String & msg);
}