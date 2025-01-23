#include "auto_parking.hpp"

using namespace std::chrono_literals;



AutoParking::AutoParking() 
    : Node("auto_parking")
{
    m_publishing = false;
    start = false;
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
    subscription_start_parking_ = this->create_subscription<std_msgs::msg::Int32>("start_parking", 10, std::bind(&AutoParking::init_parking, this, _1));
      
    timer_ = this->create_wall_timer(50ms, std::bind(&AutoParking::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "auto_parking node READY");
}

void AutoParking::init_parking(const std_msgs::msg::Int32 & i)
{

    if(i.data == static_cast<int32_t>(ParkingType::PERPENDICULAR)){ // STRAIGHT PARKING
        RCLCPP_INFO(this->get_logger(), "START Straight Parking");
        start = true;
        m_parking_type = ParkingType::PERPENDICULAR;
    } else if (i.data == static_cast<int32_t>(ParkingType::PARALLEL)) {
        RCLCPP_INFO(this->get_logger(), "START Parallel Parking");
        start = true;
        m_parking_type = ParkingType::PARALLEL;
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
            case ParkingType::PERPENDICULAR:
                m_state = ParkingState::FORWARD_15CM;
                break;
                RCLCPP_INFO(this->get_logger(), "NEW STATE ===> FORWARD_15CM, Distance: %f", ParkingDistances[static_cast<int>(m_state)]);            
            
            case ParkingType::PARALLEL:
                m_state = ParkingState::REVERSE_1M;
                break;
                RCLCPP_INFO(this->get_logger(), "NEW STATE ===> REVERSE_1M, Distance: %f", ParkingDistances[static_cast<int>(m_state)]);     
            
            default:
                break;
            }
            waiting = false;
            m_publishing = true;
        }
        break;

    // STRAIGHT PARKING

    case ParkingState::FORWARD_15CM:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move();
        }
        else if(waiting && m_current_distance >= m_current_distance_limit)
        {    
            m_state = ParkingState::REVERSE_TO_45DEG_STEER_RIGHT;
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> REVERSE_TO_45DEG_STEER_RIGHT, Distance: %f", ParkingDistances[static_cast<int>(m_state)]);
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
            m_state = ParkingState::FORWARD_60CM_STEER_LEFT_50;
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> FORWARD_60CM_STEER_LEFT_50, Distance: %f", ParkingDistances[static_cast<int>(m_state)]);
            waiting = false;
        }
        break;

    case ParkingState::FORWARD_60CM_STEER_LEFT_50:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(false, -0.75);
        }
        else if(waiting && m_current_distance >= m_current_distance_limit)
        {    
            m_state = ParkingState::REVERSE_50CM_STEER_RIGHT_40;
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> REVERSE_50CM_STEER_RIGHT_40, Distance: %f", ParkingDistances[static_cast<int>(m_state)]);
            waiting = false;
        }
        break;

    case ParkingState::REVERSE_50CM_STEER_RIGHT_40:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(true, 0.65);
        }
        else if(waiting && m_current_distance >= m_current_distance_limit)
        {    
            m_state = ParkingState::STRAIGHTEN_WHEELS_STRAIGHT;
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> STRAIGHTEN_WHEELS_STRAIGHT, Distance: %f", ParkingDistances[static_cast<int>(m_state)]);
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
            m_state = ParkingState::FINAL_REVERSE_80CM;
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> FINAL_REVERSE_80CM, Distance: %f", ParkingDistances[static_cast<int>(m_state)]);
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
            m_state = ParkingState::PARKED;
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> PARKED, Distance: %f", ParkingDistances[static_cast<int>(m_state)]);
            waiting = false;
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
            std_msgs::msg::Bool parking_finished;
            parking_finished.data = true; 
            publisher_parking_finished_->publish(parking_finished);

            m_state = ParkingState::IDLE;

            m_publishing = false;
            start = false;
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
            m_state = ParkingState::STOP_STEER_LEFT_100;
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> STOP_STEER_LEFT_100, Distance: %f", ParkingDistances[static_cast<int>(m_state)]);
            waiting = false;
        }
        break;

    case ParkingState::STOP_STEER_LEFT_100:
        if (!waiting)
        {
            waiting = true;
            car_move(false, -1.0, 0.0); // Braquer à fond à gauche (stopping = mouvement neutre)
        }
        else if (waiting && m_current_distance >= m_current_distance_limit)
        {
            m_state = ParkingState::FORWARD_65CM;
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> FORWARD_65CM, Distance: %f", ParkingDistances[static_cast<int>(m_state)]);
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
            m_state = ParkingState::TURN_BEFORE_REVERSE;
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> TURN_BEFORE_REVERSE, Distance: %f", ParkingDistances[static_cast<int>(m_state)]);
            waiting = false;
        }
        break;

    case ParkingState::TURN_BEFORE_REVERSE:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(false, 1.0, 0.2); // Avancer
        }
        else if (waiting && m_current_distance >= m_current_distance_limit)
        {
            m_state = ParkingState::STOP_STEER_RIGHT_100;
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> STOP_STEER_RIGHT_100, Distance: %f", ParkingDistances[static_cast<int>(m_state)]);
            waiting = false;
        }
        break;

    case ParkingState::STOP_STEER_RIGHT_100:
        if (!waiting)
        {
            waiting = true;
            car_move(false, 1.0, 0.0); // Braquer à fond à droite (stopping)
        }
        else if (waiting && m_current_distance >= m_current_distance_limit)
        {
            m_state = ParkingState::REVERSE_90CM;
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> REVERSE_90CM, Distance: %f", ParkingDistances[static_cast<int>(m_state)]);
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
            m_state = ParkingState::STRAIGHTEN_WHEELS_PARALLEL;
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> STRAIGHTEN_WHEELS_PARALLEL, Distance: %f", ParkingDistances[static_cast<int>(m_state)]);
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
        else if (waiting && m_current_distance >= m_current_distance_limit)
        {    
            m_state = ParkingState::REVERSE_20CM;
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> REVERSE_20CM, Distance: %f", ParkingDistances[static_cast<int>(m_state)]);
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
            m_state = ParkingState::STEER_LEFT_100_REVERSE_1M;
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> STEER_LEFT_100_REVERSE_1M, Distance: %f", ParkingDistances[static_cast<int>(m_state)]);
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
            m_state = ParkingState::STEER_RIGHT_100_REVERSE;
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> STEER_RIGHT_100_REVERSE, Distance: %f", ParkingDistances[static_cast<int>(m_state)]);
            waiting = false;
        }
        break;

    case ParkingState::STEER_RIGHT_100_REVERSE:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(true, 1.0, 0.2); // Braquer à droite et reculer
        }
        else if (waiting && m_current_distance >= m_current_distance_limit)
        {
            m_state = ParkingState::FORWARD_40CM;
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> FORWARD_40CM, Distance: %f", ParkingDistances[static_cast<int>(m_state)]);
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
            m_state = ParkingState::STEER_LEFT_FINAL;
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> STEER_LEFT_FINAL, Distance: %f", ParkingDistances[static_cast<int>(m_state)]);
            waiting = false;
        }
        break;


    case ParkingState::STEER_LEFT_FINAL:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(false, -1.0, 0.2); // Braquer à droite et reculer
        }
        else if (waiting && m_current_distance >= m_current_distance_limit)
        {
            m_state = ParkingState::REVERSE_STRAIGHT_30;
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> REVERSE_STRAIGHT_30, Distance: %f", ParkingDistances[static_cast<int>(m_state)]);
            waiting = false;
        }
        break;

    case ParkingState::REVERSE_STRAIGHT_30:
        if (!waiting)
        {
            waiting = true;
            m_current_distance = 0.0;
            car_move(true); // Avancer
        }
        else if (waiting && m_current_distance >= m_current_distance_limit)
        {
            m_state = ParkingState::PARKED;
            RCLCPP_INFO(this->get_logger(), "NEW STATE ===> PARKED, Distance: %f", ParkingDistances[static_cast<int>(m_state)]);
            waiting = false;
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
