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
        BACKWARD_STRAIGHT_25,
        FORWARD_LEFT_35,
        FORWARD_RIGHT_10_1,
        REVERSE_RIGHT_50,
        REVERSE_LEFT_5,
        FORWARD_LEFT_80,
        FORWARD_RIGHT_10_2,
        FORWARD_RIGHT_130,
        TURN_RIGHT_2_LEFT,
        BACKWARD_LEFT_40,
        TURN_LEFT_2_RIGHT,
        FORWARD_RIGHT_60

    };

    static constexpr double LeavingDistances[] = {
        
        // SHARED 
        0.0,    // IDLE
        0.0,    // LEFT_PARKING_SPACE

        // STRAIGHT
        0.0,    // STRAIGHTEN_WHEELS
        15.0,   // BEGIN_FORWARD_80CM
        80.0,   // FORWARD_80CM_STEER_RIGHT
        20.0,   // CHANGE_WHEELS
        40.0,   // REVERSE_60CM_STEER_HALF_LEFT
        60.0,   // FINAL_FORWARD_125CM_STEER_RIGHT
       

        // PARALLEL
        21.0*DISTANCE2PULSE,   // BACKWARD_STRAIGHT_25
        55.0*DISTANCE2PULSE,   // FORWARD_LEFT_35
        10.0*DISTANCE2PULSE,   // FORWARD_RIGHT_10_1
        65.0*DISTANCE2PULSE,   // REVERSE_RIGHT_50
        10.0*DISTANCE2PULSE,   // REVERSE_LEFT_5
        100.0*DISTANCE2PULSE,  // FORWARD_LEFT_80
        20.0*DISTANCE2PULSE,   // FORWARD_RIGHT_10_2
        20.0*DISTANCE2PULSE,   // FORWARD_RIGHT_130
        10.0*DISTANCE2PULSE,   // TURN_RIGHT_2_LEFT
        50.0*DISTANCE2PULSE,   // BACKWARD_LEFT_40
        10.0*DISTANCE2PULSE,   // TURN_LEFT_2_RIGHT
        135.0*DISTANCE2PULSE,   // FORWARD_RIGHT_60
    };

    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr publisher_car_order_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_leaving_finished_;

    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr motors_feedback_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_start_leaving_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    interfaces::msg::JoystickOrder car_order;

    bool m_publishing, start, waiting;
    float m_current_distance, m_current_distance_limit;

    void init_leaving(const std_msgs::msg::Int32 & i);
    void update_state(const interfaces::msg::MotorsFeedback::SharedPtr msg);
    void timer_callback();
    void car_move(bool reverse = REVERSE, float steer = STEER_NEUTRAL, float speed = SPEED_NORMAL);
    void car_stop();

    LeavingState m_state;
    ParkingType m_parking_type;
    
};

#endif // AUTO_LEAVING_HPP