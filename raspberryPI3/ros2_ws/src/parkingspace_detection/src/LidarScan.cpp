#include "/home/pi/ros2_ws/src/parkingspace_detection/include/LidarScan.hpp" 
#include <cstring>
#include <stdexcept>


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

float LidarScan::get_angle_increment() const { return angle_increment; }
void LidarScan::set_angle_increment(float value) { angle_increment = value; }

float LidarScan::get_time_increment() const { return time_increment; }
void LidarScan::set_time_increment(float value) { time_increment = value; }

float LidarScan::get_scan_time() const { return scan_time; }
void LidarScan::set_scan_time(float value) { scan_time = value; }

float LidarScan::get_range_min() const { return range_min; }
void LidarScan::set_range_min(float value) { range_min = value; }

float LidarScan::get_range_max() const { return range_max; }
void LidarScan::set_range_max(float value) { range_max = value; }

uint32_t LidarScan::get_nb_points() const { return nb_points; }

float *LidarScan::get_ranges() const { return ranges; }
float *LidarScan::get_intensities() const { return intensities; }

// Modificateurs des tableaux
void LidarScan::set_ranges(const float *new_ranges) {
    if (new_ranges) {
        // Copie les données dans ranges en limitant à NB_POINTS
        std::copy(new_ranges, new_ranges + NB_POINTS, ranges);
    }
}

void LidarScan::set_intensities(const float *new_intensities) {
    if (new_intensities) {
        // Copie les données dans intensities en limitant à NB_POINTS
        std::copy(new_intensities, new_intensities + NB_POINTS, intensities);
    }
}

int LidarScan::rechercherMin(float tableau[]) {
    float min = LIMITE_MAXI;
    int indexMin = -1;

    for (int i = 0; i < NB_POINTS; i++) {
        if (tableau[i] < min) {
            min = tableau[i];
            indexMin = i;
        }
    }

    return indexMin;
}

float LidarScan::get_range_at(uint32_t i) const {
    if (i >= NB_POINTS) {
        throw std::out_of_range("Index out of bounds in get_range_at");
    }
    return ranges[i];
}

float LidarScan::get_intensity_at(uint32_t i) const {
    if (i >= NB_POINTS) {
        throw std::out_of_range("Index out of bounds in get_intensity_at");
    }
    return intensities[i];
}
