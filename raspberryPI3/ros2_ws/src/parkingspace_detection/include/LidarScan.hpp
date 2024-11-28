#ifndef LIDARSCAN_HPP
#define LIDARSCAN_HPP

#include <cstdint>
#include <algorithm>
#define NB_POINTS 1024

class LidarScan {
public:
    // Constructeur par défaut
    LidarScan(uint32_t points = NB_POINTS);

    // Destructeur
    ~LidarScan();

    // Accesseurs
    float LidarScan::get_angle_min() const;
    void LidarScan::set_angle_min(float value);

    float LidarScan::get_angle_max() const;
    void LidarScan::set_angle_max(float value);

    float LidarScan::get_angle_increment() const;
    void LidarScan::set_angle_increment(float value);

    float LidarScan::get_time_increment() const;
    void LidarScan::set_time_increment(float value);

    float LidarScan::get_scan_time() const;
    void LidarScan::set_scan_time(float value);

    float LidarScan::get_range_min() const;
    void LidarScan::set_range_min(float value);

    float LidarScan::get_range_max() const;
    void LidarScan::set_range_max(float value);

    float *LidarScan::get_ranges() const;
    float *LidarScan::get_intensities() const;

    uint32_t LidarScan::get_nb_points() const;

private:
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;
    float *ranges;
    float *intensities;
    uint32_t nb_points;
};

#endif // LIDARSCAN_HPP
