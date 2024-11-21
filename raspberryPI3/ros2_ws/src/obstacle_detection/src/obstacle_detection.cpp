#include "obstacle_detection.hpp"

ObstacleDetection::ObstacleDetection() 
    : Node("obstacle_detection"), will_send_speed_(false), speed_value_front(SpeedCoefficient::NORMAL), speed_value_back(SpeedCoefficient::NORMAL)
{
    // Create a subscription to the "/us_data" topic
    subscription_ = this->create_subscription<interfaces::msg::Ultrasonic>(
        "/us_data", 10, std::bind(&ObstacleDetection::topic_callback, this, std::placeholders::_1));
    
    publisher_ = this->create_publisher<interfaces::msg::SpeedInfo>("/speed_info", 10);
    
    timer_ = this->create_wall_timer(50ms, std::bind(&ObstacleDetection::timer_callback, this));
}

void ObstacleDetection::timer_callback()
{
    if (will_send_speed_)
    {
        auto message = interfaces::msg::SpeedInfo();
        message.speed_coeff_front = speed_value_front;
        message.speed_coeff_back = speed_value_back;
        RCLCPP_INFO(this->get_logger(), "Publishing: [back] '%f' - [front] '%f'", message.speed_coeff_back, message.speed_coeff_front);
        publisher_->publish(message);
        will_send_speed_ = false;
    }
}

void ObstacleDetection::update_speed_info(bool is_front, int16_t sensor_value)
{
    std::string orientation = is_front ? "[FRONT]" : "[BACK]";

    if (sensor_value < THRESHOLD_STOP)
    {
        RCLCPP_WARN(this->get_logger(), "'%s' STOP !!!", orientation.c_str());
        will_send_speed_ = true;
        if (is_front)
            speed_value_front = SpeedCoefficient::STOP; 
        else
            speed_value_back = SpeedCoefficient::STOP; 
    }
    else if (sensor_value < THRESHOLD_SLOW)
    {
        RCLCPP_INFO(this->get_logger(), "'%s' SLOW DOWN AGAIN !!!", orientation.c_str());
        will_send_speed_ = true;
        if (is_front)
            speed_value_front = SpeedCoefficient::WALKING_PACE; 
        else
            speed_value_back = SpeedCoefficient::WALKING_PACE; 
    }
    else if (sensor_value < THRESHOLD_FIRST_SLOW)
    {
        RCLCPP_INFO(this->get_logger(), "'%s' SLOW DOWN BOY !!!", orientation.c_str());
        will_send_speed_ = true;
        if (is_front)
            speed_value_front = SpeedCoefficient::HALF_SPEED; 
        else
            speed_value_back = SpeedCoefficient::HALF_SPEED; 
    }
    else if (sensor_value < THRESHOLD_CAREFUL)
    {
        RCLCPP_INFO(this->get_logger(), "'%s' SOMETHING DETECTED, CAREFUL !!!", orientation.c_str());
        will_send_speed_ = true;
        if (is_front)
            speed_value_front = SpeedCoefficient::NORMAL; 
        else
            speed_value_back = SpeedCoefficient::NORMAL; 
    }
    else
    {
        // No warning or message; speed stays NORMAL.
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
