#include "lidarscan.hpp"
#include <cstring>

// Constructeur par défaut
LidarScan::LidarScan(uint32_t points)
    : angle_min(0.0f), angle_max(0.0f), angle_increment(0.0f),
      time_increment(0.0f), scan_time(0.0f), range_min(0.0f),
      range_max(0.0f), nb_points(points) {
    ranges = new float[nb_points];
    intensities = new float[nb_points];
}

// Destructeur
LidarScan::~LidarScan() {
    delete[] ranges;
    delete[] intensities;
}


// Accesseurs
float LidarScan::get_angle_min() const { return angle_min; }
void LidarScan::set_angle_min(float value) { angle_min = value; }

float LidarScan::get_angle_max() const { return angle_max; }
void LidarScan::set_angle_max(float value) { angle_max = value; }

uint32_t LidarScan::get_nb_points() const { return nb_points; }

float *LidarScan::get_ranges() const { return ranges; }
float *LidarScan::get_intensities() const { return intensities; }
