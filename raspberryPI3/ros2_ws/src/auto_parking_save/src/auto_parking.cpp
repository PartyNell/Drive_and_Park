#include "auto_parking.hpp"



/*
- quand trouvée, avancer de 15cm
- braquer 100% à droite
- reculer jusqu’à être à 45° ⇒ 1.25m
- braquer à gauche à 50%
- avancer pendant 60cm
- braquer à 100% à droite
- reculer de  80 cm
- mettre les roues droites
- reculer de 80cm
*/


AutoParking::AutoParking() 
    : Node("auto_parking")
{
    publishing = false;
    start = true;
    waiting = false;
    current_distance = 0.0;
    state = ParkingState::IDLE;
    current_distance_limit = ParkingDistances[static_cast<int>(current_state)];
    
    // Create a subscription to the "/us_data" topic
    motors_feedback_subscription_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
        "/motors_feedback", 10, std::bind(&AutoParking::update_state, this, std::placeholders::_1));
    
    publisher_car_order_ = this->create_publisher<interfaces::msg::JoystickOrder>("autonomous_car_order", 10);    
    
    timer_ = this->create_wall_timer(50ms, std::bind(&AutonomousDriving::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "auto_parking node READY");
}

void AutoParking::update_state(const interfaces::msg::MotorsFeedback::SharedPtr msg)
{
    float odometry = msg->left_rear_odometry;
    current_distance += odometry;

    // update state
    switch (state)
    {
    case ParkingState::IDLE:
        if (start)
        {
            state = ParkingState::FORWARD_15CM;
            waiting = false;
        }
        break;

    case ParkingState::FORWARD_15CM:
        if (!waiting)
        {
            waiting = true;
            current_distance = 0.0;
            car_move();
        }
        else if(waiting && current_distance >= current_distance_limit)
        {
            state = ParkingState::REVERSE_TO_45DEG_STEER_RIGHT;
            waiting = false;
        }
        break;

    case ParkingState::REVERSE_TO_45DEG_STEER_RIGHT:
        if (!waiting)
        {
            waiting = true;
            current_distance = 0.0;
            car_move(true, 1.0);
        }
        else if(waiting && current_distance >= current_distance_limit)
        {
            state = ParkingState::FORWARD_60CM_STEER_LEFT_50;
            waiting = false;
        }
        break;

    case ParkingState::FORWARD_60CM_STEER_LEFT_50:
        if (!waiting)
        {
            waiting = true;
            current_distance = 0.0;
            car_move(false, -0.5);
        }
        else if(waiting && current_distance >= current_distance_limit)
        {
            state = ParkingState::REVERSE_80CM_STEER_RIGHT_100;
            waiting = false;
        }
        break;

    case ParkingState::REVERSE_80CM_STEER_RIGHT_100:
        if (!waiting)
        {
            waiting = true;
            current_distance = 0.0;
            car_move(true, 1.0);
        }
        else if(waiting && current_distance >= current_distance_limit)
        {
            state = ParkingState::STRAIGHTEN_WHEELS;
            waiting = false;
        }
        break;

    case ParkingState::STRAIGHTEN_WHEELS:
        if (!waiting)
        {
            waiting = true;
            current_distance = 0.0;
            car_move(false, 0.0, 0.0);
        }
        else if(waiting)
        {
            state = ParkingState::FINAL_REVERSE_80CM;
            waiting = false;
        }
        break;

    case ParkingState::FINAL_REVERSE_80CM:
        if (!waiting)
        {
            waiting = true;
            current_distance = 0.0;
            car_move(true);
        }
        else if(waiting && current_distance >= current_distance_limit)
        {
            state = ParkingState::IDLE;
            waiting = false;
        }
        break;

    
    default:
        break;
    }

    current_distance_limit = ParkingDistances[static_cast<int>(current_state)];

    // process states

}

void timer_callback()
{
    if (publishing)
    {
        publisher_car_order_->publish(car_order);
    }
}

void AutoParking::car_move(bool reverse = false, float steer = 0.0, float speed = 0.3)
{
    car_order.start = true;
    car_order.mode = 1;
    car_order.throttle = speed;
    car_order.steer = steer;
    car_order.reverse = reverse;
    publishing = true;
}

void AutoParking::car_stop()
{
    car_order.start = true;
    car_order.mode = 1;
    car_order.throttle = 0.0;
    car_order.steer = 0.0;
    car_order.reverse = false;
    publishing = true;
}

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Create and run the node
    rclcpp::spin(std::make_shared<AutoParking>());

    // Shutdown the ROS 2 system
    rclcpp::shutdown();
    return 0;
}
