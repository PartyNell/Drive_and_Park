#ifndef LIDARSCAN_HPP
#define LIDARSCAN_HPP

#include <cstdint>
#include <algorithm>

#define NB_POINTS 1024
#define LIMIT_MAXI 15
#define LIMIT_INIT 50	// First 50 values for the initialization of the ref distance
#define LENGHT_CAR  0.495
#define LIMIT_DISTANCE 0.15

class LidarScan {
public:
    // Constructeur par défaut
    LidarScan(uint32_t points = NB_POINTS);

    // Destructeur
    ~LidarScan();

    // Accesseurs
    bool get_isInitialized() const;
    void set_isInitialized(bool value);

    bool get_isDetected() const;
    void set_isDetected(bool value);

    int get_init_compteur() const;
	void set_init_compteur(int value);
	
	float get_ref_distance() const;
	void set_ref_distance(float value);
    
    float get_ref_distance_init() const;
	void set_ref_distance_init(float value);

    float get_place_distance() const;
    void set_place_distance(float value);

    float get_angle_min() const;
    void set_angle_min(float value);

    float get_angle_max() const;
    void set_angle_max(float value);

    float get_angle_increment() const;
    void set_angle_increment(float value);

    float get_time_increment() const;
    void set_time_increment(float value);

    float get_scan_time() const;
    void set_scan_time(float value);

    float get_range_min() const;
    void set_range_min(float value);

    float get_range_max() const;
    void set_range_max(float value);

    uint32_t get_nb_points() const;

    // Accès aux tableaux
    float *get_ranges() const;
    float *get_intensities() const;
    float get_range_at(uint32_t i) const;
    float get_intensity_at(uint32_t i) const;

    // Modificateurs des tableaux
    void set_ranges(const float *new_ranges);
    void set_intensities(const float *new_intensities);

    int rechercherMin(float tableau[]);

private:

    bool isInitialized;
    bool isDetected;

    int init_compteur;
	float ref_distance_init;
    float ref_distance;
    float place_distance;

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
