#ifndef AUTO_LEAVING_HPP
#define AUTO_LEAVING_HPP

#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/joystick_order.hpp"
#include "interfaces/msg/speed_info.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

#include "../../parkingspace_detection/include/variable.hpp"


#define STEER_RIGHT 1.0 
#define STEER_LEFT -1.0
#define STEER_HALF_LEFT -0.5
#define STEER_NEUTRAL 0.0
#define SPEED_TEST 0.25
#define SPEED_NORMAL 0.3
#define SPEED_STOP 0.0
#define FORWARD false
#define REVERSE true


class AutoLeaving : public rclcpp::Node
{
public:
    AutoLeaving();

private:
    // Énumération pour représenter les états de l'algorithme de Leaving automatique
    enum class LeavingState
    {
        // Change the state
        /*
        IDLE,               // En attente
        FORWARD_15CM,       // Avancer de 15 cm
        REVERSE_TO_45DEG_STEER_RIGHT,   // Braquer à 100% à droite et Reculer jusqu'à être à 45° (1.25m)
        FORWARD_60CM_STEER_LEFT_50,       //  Braquer à 50% à gauche et Avancer de 60 cm
        REVERSE_80CM_STEER_RIGHT_100,       // Braquer à 100% à droite et Reculer de 80 cm
        STRAIGHTEN_WHEELS,  // Mettre les roues droites
        FINAL_REVERSE_80CM  // Reculer de 80 cm (final)
        */
        IDLE,
        STRAIGHTEN_WHEELS,
        BEGIN_FORWARD_80CM,
        FORWARD_80CM_STEER_RIGHT,
        CHANGE_WHEELS,
        REVERSE_60CM_STEER_HALF_LEFT,
        FINAL_FORWARD_125CM_STEER_RIGHT,
        LEFT_PARKING_SPACE
    };

    static constexpr double LeavingDistances[] = {
        //Change the value here also
        /*
        0.0,    // IDLE
        15.0,   // FORWARD_15CM
        125.0,  // REVERSE_TO_45DEG_STEER_RIGHT
        60.0,   // FORWARD_60CM_STEER_LEFT_50
        80.0,   // REVERSE_80CM_STEER_RIGHT_100
        0.0,    // STRAIGHTEN_WHEELS
        80.0    // FINAL_REVERSE_80CM
        */
        
        0.0,    // IDLE
        0.0,   // STRAIGHTEN_WHEELS
        15.0,   // BEGIN_FORWARD_80CM
        80.0,  // FORWARD_80CM_STEER_RIGHT
        20.0, // CHANGE_WHEELS
        40.0,   // REVERSE_60CM_STEER_HALF_LEFT
        60.0,   // FINAL_FORWARD_125CM_STEER_RIGHT
        0.0
    };

    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr publisher_car_order_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_leaving_finished_;

    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr motors_feedback_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_start_leaving_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    interfaces::msg::JoystickOrder car_order;

    bool m_publishing, start_straight, start_parallel, waiting;
    float m_current_distance, m_current_distance_limit;

    void init_leaving(const std_msgs::msg::Int32 & i);
    void update_state(const interfaces::msg::MotorsFeedback::SharedPtr msg);
    void timer_callback();
    void car_move(bool reverse = REVERSE, float steer = STEER_NEUTRAL, float speed = SPEED_NORMAL);
    void car_stop();

    LeavingState m_state;
    
};

#endif // AUTO_LEAVING_HPP