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
    // Énumération pour représenter les états de l'algorithme de parking automatique
    enum class ParkingState
    {
        IDLE,               // En attente
        FORWARD_15CM,       // Avancer de 15 cm
        REVERSE_TO_45DEG_STEER_RIGHT,   // Braquer à 100% à droite et Reculer jusqu'à être à 45° (1.25m)
        FORWARD_60CM_STEER_LEFT_50,       //  Braquer à 50% à gauche et Avancer de 60 cm
        REVERSE_50CM_STEER_RIGHT_20,       // Braquer à 10% à droite et Reculer de 80 cm
        STRAIGHTEN_WHEELS,  // Mettre les roues droites
        FINAL_REVERSE_80CM,  // Reculer de 80 cm (final)
        PARKED
    };

    static constexpr double ParkingDistances[] = {
        0.0,    // IDLE
        15.0,   // FORWARD_15CM
        125.0,  // REVERSE_TO_45DEG_STEER_RIGHT
        60.0,   // FORWARD_60CM_STEER_LEFT_50
        50.0,   // REVERSE_50CM_STEER_RIGHT_20
        0.0,    // STRAIGHTEN_WHEELS
        80.0,    // FINAL_REVERSE_80CM
        0.0
    };



    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr publisher_car_order_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_parking_finished_;
    
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr motors_feedback_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_start_parking_;
    rclcpp::TimerBase::SharedPtr timer_;
    interfaces::msg::JoystickOrder car_order;
    bool m_publishing, start, waiting;
    void init_parking(const std_msgs::msg::Bool & i);
    void update_state(const interfaces::msg::MotorsFeedback::SharedPtr msg);
    void timer_callback();
    void car_move(bool reverse = false, float steer = 0.0, float speed = 0.3);
    void car_stop();
    ParkingState m_state;
    float m_current_distance, m_current_distance_limit;
};

#endif // AUTO_PARKING_HPP
