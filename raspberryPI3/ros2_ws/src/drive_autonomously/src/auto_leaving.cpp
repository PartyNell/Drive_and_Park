#include "auto_leaving.hpp"

using namespace std::chrono_literals;

/*
- mettre les roues droites
- avancer de 80cm
- braquer à 100% à droite
- avancer de 80cm
- braquer à gauche à 50%
- reculer pendant 60cm (45°)
- braquer 100% à droite
- avancer jusqu’à être droit  ⇒ 125cm
*/

AutoLeaving::AutoLeaving() 
    : Node("auto_leaving")
{
    m_publishing = false;
    start = false;
    waiting = false;
    m_current_distance = 0.0;

    // A INIT SELON L'ORDREs
    m_parking_type = ParkingType::PARALLEL;

    m_state = LeavingState::IDLE;
    m_current_distance_limit = LeavingDistances[static_cast<int>(m_state)];
    
    publisher_car_order_ = this->create_publisher<interfaces::msg::JoystickOrder>("autonomous_car_order", 10);
    publisher_leaving_finished_ = this->create_publisher<std_msgs::msg::Bool>("leaving_finished", 10);

    // Create a subscription to the "/us_data" topic
    motors_feedback_subscription_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
        "/motors_feedback", 10, std::bind(&AutoLeaving::update_state, this, std::placeholders::_1));
    
    subscription_start_leaving_ = this->create_subscription<std_msgs::msg::Int32>("start_leaving", 10, std::bind(&AutoLeaving::init_leaving, this, std::placeholders::_1));
       
    timer_ = this->create_wall_timer(50ms, std::bind(&AutoLeaving::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "auto_leaving node READY");
}

void AutoLeaving::init_leaving(const std_msgs::msg::Int32 & i)
{

    if(i.data == static_cast<int32_t>(ParkingType::PERPENDICULAR)){ // STRAIGHT PARKING
        RCLCPP_INFO(this->get_logger(), "START Straight Leaving");
        start = true;
        m_parking_type = ParkingType::PERPENDICULAR;
    } else if (i.data == static_cast<int32_t>(ParkingType::PARALLEL)) {
        RCLCPP_INFO(this->get_logger(), "START Parallel Leaving");
        start = true;
        m_parking_type = ParkingType::PARALLEL;
    } else {
        RCLCPP_INFO(this->get_logger(), "STOP Parking");
        m_state = LeavingState::IDLE;
        m_current_distance = 0.0;
        start = false;
        waiting = false;
    }
}

void AutoLeaving::update_state(const interfaces::msg::MotorsFeedback::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Updating...");
    float odometry = msg->left_rear_odometry;
    m_current_distance += odometry;

    // update state
    switch (m_state)
    {
    case LeavingState::IDLE:
        if (start)
        {    
            switch (m_parking_type)
            {
            case ParkingType::PERPENDICULAR:
                m_state = LeavingState::STRAIGHTEN_WHEELS;
                break;
                RCLCPP_INFO(this->get_logger(), "NEW STATE ===> STRAIGHTEN_WHEELS, Distance: %f", LeavingDistances[static_cast<int>(m_state)]);            
            
            case ParkingType::PARALLEL:
                m_state = LeavingState::BACKWARD_STRAIGHT_25;
                break;
                RCLCPP_INFO(this->get_logger(), "NEW STATE ===> BACKWARD_STRAIGHT_25, Distance: %f", LeavingDistances[static_cast<int>(m_state)]);     
            
            default:
                break;
            }
            waiting = false;
        }
        break;

        

    case LeavingState::STRAIGHTEN_WHEELS:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(FORWARD, STEER_NEUTRAL, SPEED_STOP);
        }
        else if(waiting)
        {    
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> BEGIN_FORWARD_80CM");
            m_state = LeavingState::BEGIN_FORWARD_80CM;
            waiting = false;
        }
        break;

    case LeavingState::BEGIN_FORWARD_80CM:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(FORWARD, STEER_NEUTRAL, SPEED_NORMAL); 
        }
        else if(waiting && m_current_distance >= m_current_distance_limit)
        {    
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> FORWARD_80CM_STEER_RIGHT");
            m_state = LeavingState::FORWARD_80CM_STEER_RIGHT;
            waiting = false;
        }
        break;

    case LeavingState::FORWARD_80CM_STEER_RIGHT:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(FORWARD, STEER_RIGHT, SPEED_SLOW);
        }
        else if(waiting && m_current_distance >= m_current_distance_limit)
        {    
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> CHANGE_WHEELS");
            m_state = LeavingState::CHANGE_WHEELS;
            waiting = false;
        }
        break;
    
    case LeavingState::CHANGE_WHEELS:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(FORWARD, STEER_LEFT, SPEED_SLOW);
        }
        else if(waiting && m_current_distance >= m_current_distance_limit)
        {    
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> REVERSE_60CM_STEER_HALF_LEFT");
            m_state = LeavingState::REVERSE_60CM_STEER_HALF_LEFT;
            waiting = false;
        }
        break;

    case LeavingState::REVERSE_60CM_STEER_HALF_LEFT:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(REVERSE, STEER_LEFT, SPEED_SLOW);
        }
        else if(waiting && m_current_distance >= m_current_distance_limit)
        {    
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> FINAL_FORWARD_125CM_STEER_RIGHT");
            m_state = LeavingState::FINAL_FORWARD_125CM_STEER_RIGHT;
            waiting = false;
        }
        break;

    case LeavingState::FINAL_FORWARD_125CM_STEER_RIGHT:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(FORWARD, STEER_RIGHT, SPEED_NORMAL);
        }
        else if(waiting && m_current_distance >= m_current_distance_limit)
        {    
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> LEFT_PARKING_SPACE");
            m_state = LeavingState::LEFT_PARKING_SPACE;
            waiting = false;

            std_msgs::msg::Bool leaving_finished;
            leaving_finished.data = true; 
            publisher_leaving_finished_->publish(leaving_finished);
        }
        break;

    case LeavingState::LEFT_PARKING_SPACE:
        if (!waiting)
        {
            waiting = true;
            RCLCPP_INFO(this->get_logger(), "CAR IS READY TO GO ! HAVE A NICE TRAVEL !");
            car_move(false, 0.0, 0.0);
        }
        else if(waiting)
        {    
            std_msgs::msg::Bool leaving_finished;
            leaving_finished.data = true; 
            publisher_leaving_finished_->publish(leaving_finished);

            m_state = LeavingState::IDLE;

            m_publishing = false;
            start = false;
        }
        break;

    // PARALLEL

    case LeavingState::BACKWARD_STRAIGHT_25:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(REVERSE, STEER_NEUTRAL, SPEED_NORMAL);
        }
        else if(waiting && m_current_distance >= m_current_distance_limit)
        {    
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> FORWARD_LEFT_35");
            m_state = LeavingState::FORWARD_LEFT_35;
            waiting = false;
        }
        break;

    case LeavingState::FORWARD_LEFT_35:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(FORWARD, STEER_LEFT, SPEED_NORMAL);
        }
        else if(waiting && m_current_distance >= m_current_distance_limit)
        {    
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> FORWARD_RIGHT_10_1");
            m_state = LeavingState::FORWARD_RIGHT_10_1;
            waiting = false;
        }
        break;
    
    case LeavingState::FORWARD_RIGHT_10_1:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(FORWARD, STEER_RIGHT, SPEED_SLOW);
        }
        else if(waiting && m_current_distance >= m_current_distance_limit)
        {    
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> REVERSE_RIGHT_50");
            m_state = LeavingState::REVERSE_RIGHT_50;
            waiting = false;
        }
        break;

    case LeavingState::REVERSE_RIGHT_50:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(REVERSE, STEER_RIGHT, SPEED_NORMAL);
        }
        else if(waiting && m_current_distance >= m_current_distance_limit)
        {    
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> REVERSE_LEFT_5");
            m_state = LeavingState::REVERSE_LEFT_5;
            waiting = false;
        }
        break;

    case LeavingState::REVERSE_LEFT_5:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(REVERSE, STEER_LEFT, SPEED_SLOW);
        }
        else if(waiting && m_current_distance >= m_current_distance_limit)
        {    
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> FORWARD_LEFT_80");
            m_state = LeavingState::FORWARD_LEFT_80;
            waiting = false;
        }
        break;

    case LeavingState::FORWARD_LEFT_80:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(FORWARD, STEER_LEFT, SPEED_NORMAL);
        }
        else if(waiting && m_current_distance >= m_current_distance_limit)
        {    
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> FORWARD_RIGHT_10_2");
            m_state = LeavingState::FORWARD_RIGHT_10_2;
            waiting = false;
        }
        break;

    case LeavingState::FORWARD_RIGHT_10_2:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(FORWARD, STEER_RIGHT, SPEED_SLOW);
        }
        else if(waiting && m_current_distance >= m_current_distance_limit)
        {    
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> FORWARD_RIGHT_130");
            m_state = LeavingState::FORWARD_RIGHT_130;
            waiting = false;
        }
        break;

    case LeavingState::FORWARD_RIGHT_130:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(FORWARD, STEER_RIGHT, SPEED_NORMAL);
        }
        else if(waiting && m_current_distance >= m_current_distance_limit)
        {    
            RCLCPP_INFO(this->get_logger(), "CAR IS READY TO GO ! HAVE A NICE TRAVEL !");
            std_msgs::msg::Bool leaving_finished;
            leaving_finished.data = true; 
            publisher_leaving_finished_->publish(leaving_finished);

            m_state = LeavingState::IDLE;

            m_publishing = false;
            start = false;
        }
        break;

    default:
        break;
    }

    m_current_distance_limit = LeavingDistances[static_cast<int>(m_state)];

}

void AutoLeaving::timer_callback()
{
    if (m_publishing)
    {
        publisher_car_order_->publish(car_order);
    }
}

void AutoLeaving::car_move(bool reverse, float steer, float speed)
{
    car_order.start = true;
    car_order.mode = 1;
    car_order.throttle = speed;
    car_order.steer = steer;
    car_order.reverse = reverse;
    m_publishing = true;
}

void AutoLeaving::car_stop()
{
    car_order.start = true;
    car_order.mode = 1;
    car_order.throttle = 0.0;
    car_order.steer = 0.0;
    car_order.reverse = false;
    m_publishing = true;
}

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Create and run the node
    rclcpp::spin(std::make_shared<AutoLeaving>());

    // Shutdown the ROS 2 system
    rclcpp::shutdown();
    return 0;
}