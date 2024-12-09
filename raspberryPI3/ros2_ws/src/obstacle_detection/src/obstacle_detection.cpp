#include "obstacle_detection.hpp"

ObstacleDetection::ObstacleDetection() 
    : Node("obstacle_detection"), will_send_speed_(false), speed_value_front(SpeedCoefficient::NORMAL), 
        speed_value_back(SpeedCoefficient::NORMAL), parkmod_(true)
{
    // Create a subscription to the "/us_data" topic
    subscription_ = this->create_subscription<interfaces::msg::Ultrasonic>(
        "/us_data", 10, std::bind(&ObstacleDetection::topic_callback, this, std::placeholders::_1));
    
    publisher_ = this->create_publisher<interfaces::msg::SpeedInfo>("/speed_info", 10);
    
    timer_ = this->create_wall_timer(50ms, std::bind(&ObstacleDetection::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "obstacle_detection node READY");

    //////////////////////////////////////////////////////////////////////
    subscription_lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",  // Nom du topic
            10,       // QoS
            std::bind(
                &ObstacleDetection::laserScanCallback, this, std::placeholders::_1));
    //////////////////////////////////////////////////////////////////
}

void ObstacleDetection::timer_callback()
{
    if (will_send_speed_)
    {
        auto message = interfaces::msg::SpeedInfo();
        message.speed_coeff_front = speed_value_front;
        message.speed_coeff_back = speed_value_back;
        //RCLCPP_INFO(this->get_logger(), "Publishing: [back] '%f' - [front] '%f'", message.speed_coeff_back, message.speed_coeff_front);
        publisher_->publish(message);
        will_send_speed_ = false;
    }
}

void ObstacleDetection::update_speed_info(
    bool is_front, int16_t sensor_value)
{
    std::string orientation = is_front ? "[FRONT]" : "[BACK]";
  
///////////////////////////////////////////////////////////////////////////////////
    /*Park security
    On the "parkmod", compare ultrasonic sensor with the 15cm margin. 
    Stop the car if the direction is the same as the sensor detection
    */
    if (get_parkmod() == true && 
        sensor_value < THRESHOLD_PARK_STOP)
    {
        will_send_speed_ = true;

        if (is_front) {
            if (speed_value_front != SpeedCoefficient::STOP){
                RCLCPP_WARN(this->get_logger(), "'%s' STOP 15CM !!!", orientation.c_str());
            }
            speed_value_front = SpeedCoefficient::STOP; 
        }

        else {
            if (speed_value_back != SpeedCoefficient::STOP){
                RCLCPP_WARN(this->get_logger(), "'%s' STOP 15CM !!!", orientation.c_str());
            }
            speed_value_back = SpeedCoefficient::STOP; 
        }
    } 
////////////////////////////////////////////////////////////////////////////////
     
    if (get_parkmod() == false &&
        sensor_value < THRESHOLD_STOP)
    {
        will_send_speed_ = true;
        if (is_front){
            if (speed_value_front != SpeedCoefficient::STOP)
                RCLCPP_WARN(this->get_logger(), "'%s' STOP !!!", orientation.c_str());

            speed_value_front = SpeedCoefficient::STOP; 
        } else {
            if (speed_value_back != SpeedCoefficient::STOP)
                RCLCPP_WARN(this->get_logger(), "'%s' STOP !!!", orientation.c_str());

            speed_value_back = SpeedCoefficient::STOP; 
        }
    }
    
    else if (get_parkmod() == false &&
        sensor_value < THRESHOLD_SLOW)
    {
        will_send_speed_ = true;
        if (is_front){
            if (speed_value_front != SpeedCoefficient::WALKING_PACE)
                RCLCPP_INFO(this->get_logger(), "'%s' SLOW DOWN AGAIN !!!", orientation.c_str());

            speed_value_front = SpeedCoefficient::WALKING_PACE; 
        } else {
            if (speed_value_back != SpeedCoefficient::WALKING_PACE)
                RCLCPP_INFO(this->get_logger(), "'%s' SLOW DOWN AGAIN !!!", orientation.c_str());

            speed_value_back = SpeedCoefficient::WALKING_PACE; 
        }
    }
    
    else if (get_parkmod() == false &&
        sensor_value < THRESHOLD_FIRST_SLOW)
    {
        will_send_speed_ = true;
        if (is_front){
            if (speed_value_front != SpeedCoefficient::HALF_SPEED)
                RCLCPP_INFO(this->get_logger(), "'%s' SLOW DOWN BOY !!!", orientation.c_str());

            speed_value_front = SpeedCoefficient::HALF_SPEED; 
        } else {
            if (speed_value_back != SpeedCoefficient::HALF_SPEED)
                RCLCPP_INFO(this->get_logger(), "'%s' SLOW DOWN BOY !!!", orientation.c_str());

            speed_value_back = SpeedCoefficient::HALF_SPEED; 
        }
    }
    else if (get_parkmod() == false &&
        sensor_value < THRESHOLD_CAREFUL)
    {
        RCLCPP_INFO(this->get_logger(), "'%s' SOMETHING DETECTED, CAREFULL !!!", orientation.c_str());
        will_send_speed_ = true;
        if (is_front){
            speed_value_front = SpeedCoefficient::NORMAL; 
        } else {
            speed_value_back = SpeedCoefficient::NORMAL; 
        }
    }
    else
    {
        // No warning or message; speed stays NORMAL.
        will_send_speed_ = true;
        if (is_front)
            speed_value_front = SpeedCoefficient::NORMAL; 
        else
            speed_value_back = SpeedCoefficient::NORMAL;
    }
}

void ObstacleDetection::topic_callback(const interfaces::msg::Ultrasonic::SharedPtr msg)
{
    int16_t front_center = msg->front_center;
    int16_t rear_center = msg->rear_center;
    will_send_speed_ = false;

    update_speed_info(true, front_center);
    update_speed_info(false, rear_center);
}

///////////////////////////////////
void ObstacleDetection::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    
    RCLCPP_INFO(this->get_logger(), "In callback");

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            const auto &range = msg->ranges[i];

            if (range <= 0.3 && !std::isinf(range) && !std::isnan(range))
            {
                RCLCPP_WARN(this->get_logger(), "Range[%zu] is valid and less than 30 cm: %f", i, range);
            }
        }
}
///////////////////////////////////


int main(int argc, char *argv[])
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Create and run the node
    rclcpp::spin(std::make_shared<ObstacleDetection>());

    // Shutdown the ROS 2 system
    rclcpp::shutdown();
    return 0;
}