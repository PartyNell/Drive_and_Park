#include "auto_parking.hpp"

using namespace std::chrono_literals;



AutoParking::AutoParking() 
    : Node("auto_parking")
{
    m_publishing = false;
    start = true;
    waiting = false;
    m_current_distance = 0.0;

    // A INIT SELON L'ORDREs
    m_parking_type = ParkingType::PARALLEL;

    m_state = ParkingState::IDLE;
    m_current_distance_limit = ParkingDistances[static_cast<int>(m_state)];
    
    publisher_car_order_ = this->create_publisher<interfaces::msg::JoystickOrder>("autonomous_car_order", 10); 
    publisher_parking_finished_ = this->create_publisher<std_msgs::msg::Bool>("parking_finished", 10);

    // Create a subscription to the "/us_data" topic
    motors_feedback_subscription_ = this->create_subscription<interfaces::msg::MotorsFeedback>("/motors_feedback", 10, std::bind(&AutoParking::update_state, this, std::placeholders::_1));
    subscription_start_parking_ = this->create_subscription<std_msgs::msg::Bool>("start_parking", 10, std::bind(&AutoParking::init_parking, this, _1));

   
    timer_ = this->create_wall_timer(50ms, std::bind(&AutoParking::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "auto_parking node READY");
}

void AutoParking::init_parking(const std_msgs::msg::Bool & i)
{
    start = i.data;

    if(start){
        RCLCPP_INFO(this->get_logger(), "START Parking");
    } else {
        RCLCPP_INFO(this->get_logger(), "STOP Parking");
        m_state = ParkingState::IDLE;
        m_current_distance = 0.0;
        waiting = false;
    }
}

void AutoParking::update_state(const interfaces::msg::MotorsFeedback::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Updating...");
    float odometry = msg->left_rear_odometry;
    m_current_distance += odometry;

    // update state
    switch (m_state)
    {
    case ParkingState::IDLE:
        if (start)
        {    
            switch (m_parking_type)
            {
            case ParkingType::STRAIGHT:
                m_state = ParkingState::FORWARD_15CM;
                RCLCPP_INFO(this->get_logger(), "NEW STATE ===> FORWARD_15CM");            
                break;
            
            case ParkingType::PARALLEL:
                m_state = ParkingState::REVERSE_1M;
                RCLCPP_INFO(this->get_logger(), "NEW STATE ===> REVERSE_1M");     
                break;
            
            default:
                break;
            }
            waiting = false;
        }
        break;

    case ParkingState::FORWARD_15CM:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move();
        }
        else if(waiting && m_current_distance >= m_current_distance_limit)
        {    
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> REVERSE_TO_45DEG_STEER_RIGHT");
            m_state = ParkingState::REVERSE_TO_45DEG_STEER_RIGHT;
            waiting = false;
        }
        break;

    case ParkingState::REVERSE_TO_45DEG_STEER_RIGHT:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(true, 1.0);
        }
        else if(waiting && m_current_distance >= m_current_distance_limit)
        {    
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> FORWARD_60CM_STEER_LEFT_50");
            m_state = ParkingState::FORWARD_60CM_STEER_LEFT_50;
            waiting = false;
        }
        break;

    case ParkingState::FORWARD_60CM_STEER_LEFT_50:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(false, -0.5);
        }
        else if(waiting && m_current_distance >= m_current_distance_limit)
        {    
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> REVERSE_50CM_STEER_RIGHT_40");
            m_state = ParkingState::REVERSE_50CM_STEER_RIGHT_40;
            waiting = false;
        }
        break;

    case ParkingState::REVERSE_50CM_STEER_RIGHT_40:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(true, 0.4);
        }
        else if(waiting && m_current_distance >= m_current_distance_limit)
        {    
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> STRAIGHTEN_WHEELS_STRAIGHT");
            m_state = ParkingState::STRAIGHTEN_WHEELS_STRAIGHT;
            waiting = false;
        }
        break;

    case ParkingState::STRAIGHTEN_WHEELS_STRAIGHT:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(false, 0.0, 0.0);
        }
        else if(waiting)
        {    
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> FINAL_REVERSE_80CM");
            m_state = ParkingState::FINAL_REVERSE_80CM;
            waiting = false;
        }
        break;

    case ParkingState::FINAL_REVERSE_80CM:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(true);
        }
        else if(waiting && m_current_distance >= m_current_distance_limit)
        {    
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> PARKED");
            m_state = ParkingState::PARKED;
            waiting = false;

            std_msgs::msg::Bool parking_finished;
            parking_finished.data = true; 
            publisher_parking_finished_->publish(parking_finished);
        }
        break;

    case ParkingState::PARKED:
        if (!waiting)
        {
            waiting = true;
            RCLCPP_INFO(this->get_logger(), "CAR IS PARKED ! ENJOY !");
            car_move(false, 0.0, 0.0);
        }
        else if(waiting)
        {    
            // wait for any signal to park again
        }
        break;


    // PARALLEL

    case ParkingState::REVERSE_1M:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(true); // Reculer
        }
        else if (waiting && m_current_distance >= m_current_distance_limit)
        {
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> STOP_STEER_LEFT_100");
            m_state = ParkingState::STOP_STEER_LEFT_100;
            waiting = false;
        }
        break;

    case ParkingState::STOP_STEER_LEFT_100:
        if (!waiting)
        {
            waiting = true;
            car_move(false, -1.0); // Braquer à fond à gauche (stopping = mouvement neutre)
        }
        else if (waiting)
        {
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> FORWARD_65CM");
            m_state = ParkingState::FORWARD_65CM;
            waiting = false;
        }
        break;

    case ParkingState::FORWARD_65CM:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(false, -1.0); // Avancer
        }
        else if (waiting && m_current_distance >= m_current_distance_limit)
        {
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> STOP_STEER_RIGHT_100");
            m_state = ParkingState::STOP_STEER_RIGHT_100;
            waiting = false;
        }
        break;

    case ParkingState::STOP_STEER_RIGHT_100:
        if (!waiting)
        {
            waiting = true;
            car_move(false, 1.0); // Braquer à fond à droite (stopping)
        }
        else if (waiting)
        {
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> REVERSE_90CM");
            m_state = ParkingState::REVERSE_90CM;
            waiting = false;
        }
        break;

    case ParkingState::REVERSE_90CM:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(true, 1.0); // Reculer
        }
        else if (waiting && m_current_distance >= m_current_distance_limit)
        {
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> STRAIGHTEN_WHEELS_PARALLEL");
            m_state = ParkingState::STRAIGHTEN_WHEELS_PARALLEL;
            waiting = false;
        }
        break;

    case ParkingState::STRAIGHTEN_WHEELS_PARALLEL:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(false, 0.0, 0.0);
        }
        else if(waiting)
        {    
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> FINAL_REVERSE_80CM");
            m_state = ParkingState::FINAL_REVERSE_80CM;
            waiting = false;
        }
        break;

    case ParkingState::REVERSE_20CM:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(true); // Reculer
        }
        else if (waiting && m_current_distance >= m_current_distance_limit)
        {
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> STEER_LEFT_100_REVERSE_1M");
            m_state = ParkingState::STEER_LEFT_100_REVERSE_1M;
            waiting = false;
        }
        break;

    case ParkingState::STEER_LEFT_100_REVERSE_1M:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(true, -1.0); // Braquer à gauche et reculer
        }
        else if (waiting && m_current_distance >= m_current_distance_limit)
        {
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> STEER_RIGHT_100_REVERSE");
            m_state = ParkingState::STEER_RIGHT_100_REVERSE;
            waiting = false;
        }
        break;

    case ParkingState::STEER_RIGHT_100_REVERSE:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(true, 1.0); // Braquer à droite et reculer
        }
        else if (waiting)
        {
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> FORWARD_40CM");
            m_state = ParkingState::FORWARD_40CM;
            waiting = false;
        }
        break;

    case ParkingState::FORWARD_40CM:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(false, 1.0); // Avancer
        }
        else if (waiting && m_current_distance >= m_current_distance_limit)
        {
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> PARKED");
            m_state = ParkingState::PARKED;
            waiting = false;

            std_msgs::msg::Bool parking_finished;
            parking_finished.data = true;
            publisher_parking_finished_->publish(parking_finished);
        }
        break;


    default:
        break;
    }

    m_current_distance_limit = ParkingDistances[static_cast<int>(m_state)];

    // process states

}

void AutoParking::timer_callback()
{
    if (m_publishing)
    {
        publisher_car_order_->publish(car_order);
    }
}

void AutoParking::car_move(bool reverse, float steer, float speed)
{
    car_order.start = true;
    car_order.mode = 1;
    car_order.throttle = speed;
    car_order.steer = steer;
    car_order.reverse = reverse;
    m_publishing = true;
}

void AutoParking::car_stop()
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
    rclcpp::spin(std::make_shared<AutoParking>());

    // Shutdown the ROS 2 system
    rclcpp::shutdown();
    return 0;
}
