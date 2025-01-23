#ifndef AUTO_LEAVING_HPP
#define AUTO_LEAVING_HPP

#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/joystick_order.hpp"
#include "interfaces/msg/speed_info.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "std_msgs/msg/bool.hpp"


#define STEER_RIGHT 1.0 
#define STEER_LEFT -1.0
#define STEER_HALF_LEFT -0.5
#define STEER_NEUTRAL 0.0
#define SPEED_SLOW 0.25
#define SPEED_NORMAL 0.3
#define SPEED_STOP 0.0
#define FORWARD false
#define REVERSE true

#define DISTANCE2PULSE (36/(20*3.14))

class AutoLeaving : public rclcpp::Node
{
public:
    AutoLeaving();

private:

    enum class ParkingType
    {
        STRAIGHT,
        PARALLEL
    };
    
    // Énumération pour représenter les états de l'algorithme de Leaving automatique
    enum class LeavingState
    {
        // SHARED
        IDLE,
        LEFT_PARKING_SPACE,

        // STRAIGHT LEAVING STATE
        STRAIGHTEN_WHEELS,
        BEGIN_FORWARD_80CM,
        FORWARD_80CM_STEER_RIGHT,
        CHANGE_WHEELS,
        REVERSE_60CM_STEER_HALF_LEFT,
        FINAL_FORWARD_125CM_STEER_RIGHT,

        // PARALLEL LEAVING STATE
        REVERSE_30_STEER_RIGHT,
        FORWARD_110_STEER_LEFT,
        FORWARD_110_STEER_RIGHT
    };

    static constexpr double LeavingDistances[] = {
        
        // SHARED 
        0.0,    // IDLE
        0.0,    // LEFT_PARKING_SPACE

        // STRAIGHT
        0.0,   // STRAIGHTEN_WHEELS
        15.0,   // BEGIN_FORWARD_80CM
        80.0,  // FORWARD_80CM_STEER_RIGHT
        20.0, // CHANGE_WHEELS
        40.0,   // REVERSE_60CM_STEER_HALF_LEFT
        60.0,   // FINAL_FORWARD_125CM_STEER_RIGHT
       

        // PARALLEL
        30.0*DISTANCE2PULSE,   // REVERSE_30_STEER_RIGHT
        110.0*DISTANCE2PULSE,   // FORWARD_110_STEER_LEFT
        110.0*DISTANCE2PULSE  // FORWARD_110_STEER_RIGHT
    };

    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr publisher_car_order_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_leaving_finished_;

    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr motors_feedback_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_start_leaving_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    interfaces::msg::JoystickOrder car_order;

    bool m_publishing, start, waiting;
    float m_current_distance, m_current_distance_limit;

    void init_leaving(const std_msgs::msg::Bool & i);
    void update_state(const interfaces::msg::MotorsFeedback::SharedPtr msg);
    void timer_callback();
    void car_move(bool reverse = REVERSE, float steer = STEER_NEUTRAL, float speed = SPEED_NORMAL);
    void car_stop();

    LeavingState m_state;
    ParkingType m_parking_type;
    
};

#endif // AUTO_LEAVING_HPP