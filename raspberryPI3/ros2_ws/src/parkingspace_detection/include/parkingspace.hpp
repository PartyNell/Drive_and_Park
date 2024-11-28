#ifndef PARKINGSPACE_HPP
#define PARKINGSPACE_HPP

#include <chrono>
#include <string>
#include <stdint.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/LaserScan"

ParkingSpace : public rclcpp::Node
{
  public:
    ParkingSpace();

  private:
    typedef struct {
    float angle_min;       // Angle minimum du scan (en radians)
    float angle_max;       // Angle maximum du scan (en radians)
    float angle_increment; // Incrément d'angle entre les mesures (en radians)
    float time_increment;  // Temps entre deux mesures (en secondes)
    float scan_time;       // Temps total d'un scan complet (en secondes)
    float range_min;       // Portée minimum détectable (en mètres)
    float range_max;       // Portée maximum détectable (en mètres)
    float *ranges;         // Tableau des distances mesurées
    float *intensities;    // Tableau des intensités mesurées
    uint32_t nb_points;   // Nombre de points dans les tableaux ranges et intensities
    } LidarScan;


    void topic_callback(const std_msgs::msg::String & msg);
}