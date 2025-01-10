#ifndef AUTO_PARKING_HPP
#define AUTO_PARKING_HPP

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/joystick_order.hpp"
#include "interfaces/msg/speed_info.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;

class AutoParking : public rclcpp::Node
{
public:
    AutoParking();

private:

    enum class ParkingType
    {
        STRAIGHT,
        PARALLEL
    };


    // Énumération pour représenter les états de l'algorithme de parking automatique
    enum class ParkingState
    {
        // PARTAGE
        IDLE,               // En attente
        PARKED,             // Garée

        // STRAIGHT PARKING STATE
        FORWARD_15CM,       // Avancer de 15 cm
        REVERSE_TO_45DEG_STEER_RIGHT,   // Braquer à 100% à droite et Reculer jusqu'à être à 45° (1.25m)
        FORWARD_60CM_STEER_LEFT_50,       //  Braquer à 50% à gauche et Avancer de 60 cm
        REVERSE_50CM_STEER_RIGHT_40,       // Braquer à 10% à droite et Reculer de 80 cm
        STRAIGHTEN_WHEELS_STRAIGHT,  // Mettre les roues droites
        FINAL_REVERSE_80CM,  // Reculer de 80 cm (final)
        
        // PARALLEL PARKING STATE
        REVERSE_1M,                 // Reculer de 1m
        STOP_STEER_LEFT_100,        // Stop et braque à fond à gauche
        FORWARD_65CM,               // Avancer de 65 cm
        STOP_STEER_RIGHT_100,       // Stop et braque à fond à droite
        REVERSE_90CM,               // Reculer de 90 cm
        STRAIGHTEN_WHEELS_PARALLEL, // Mettre les roues droites
        REVERSE_20CM,               // Reculer de 20 cm
        STEER_LEFT_100_REVERSE_1M,  // Braquer à gauche et reculer de 1m
        STEER_RIGHT_100_REVERSE,    // Braquer à droite et reculer
        FORWARD_40CM                // Avancer de 40 cm
    };

    static constexpr double ParkingDistances[] = {
        // PARTAGE
        0.0,    // IDLE
        0.0,    // PARKED

        // STRAIGHT PARKING STATE
        15.0,   // FORWARD_15CM
        125.0,  // REVERSE_TO_45DEG_STEER_RIGHT
        60.0,   // FORWARD_60CM_STEER_LEFT_50
        50.0,   // REVERSE_50CM_STEER_RIGHT_40
        0.0,    // STRAIGHTEN_WHEELS_STRAIGHT
        80.0,   // FINAL_REVERSE_80CM
        
        // PARALLEL PARKING STATE
        100.0,  // REVERSE_1M
        0.0,    // STOP_STEER_LEFT_100
        65.0,   // FORWARD_65CM
        0.0,    // STOP_STEER_RIGHT_100
        90.0,   // REVERSE_90CM
        0.0,    // STRAIGHTEN_WHEELS_PARALLEL
        20.0,   // REVERSE_20CM
        100.0,  // STEER_LEFT_100_REVERSE_1M
        0.0,    // STEER_RIGHT_100_REVERSE
        40.0    // FORWARD_40CM
    };

    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr publisher_car_order_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_parking_finished_;
    
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr motors_feedback_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_start_parking_;
    rclcpp::TimerBase::SharedPtr timer_;
    interfaces::msg::JoystickOrder car_order;

    rclcpp::Clock clock_ = rclcpp::Clock(RCL_SYSTEM_TIME);

    bool m_publishing, start, waiting;
    void init_parking(const std_msgs::msg::Bool & i);
    void update_state(const interfaces::msg::MotorsFeedback::SharedPtr msg);
    void timer_callback();
    void car_move(bool reverse = false, float steer = 0.0, float speed = 0.3);
    void car_stop();
    void delay(); 

    ParkingState m_state;
    ParkingType m_parking_type;
    rclcpp::Duration waiting_delay  = rclcpp::Duration::from_seconds(5);

    float m_current_distance, m_current_distance_limit;
};

#endif // AUTO_PARKING_HPP
