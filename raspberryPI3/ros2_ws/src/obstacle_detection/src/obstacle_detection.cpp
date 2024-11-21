#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/ultrasonic.hpp"
#include "interfaces/msg/speed_info.hpp"

#define THRESHOLD_STOP 35
#define THRESHOLD_SLOW 45
#define THRESHOLD_FIRST_SLOW 70
#define THRESHOLD_CAREFUL 100

using namespace std::chrono_literals;

class ObstacleDetection : public rclcpp::Node
{
public:
    ObstacleDetection() : Node("obstacle_detection")
    {
        // Create a subscription to the "/us_data" topic
        subscription_ = this->create_subscription<interfaces::msg::Ultrasonic>("/us_data", 10, std::bind(&ObstacleDetection::topic_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<interfaces::msg::SpeedInfo>("/speed_info", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&ObstacleDetection::timer_callback, this));
    }

private:

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interfaces::msg::SpeedInfo>::SharedPtr publisher_;
    rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr subscription_;
    size_t count_;

    bool will_send_speed_;
    float speed_value_front, speed_value_back;

    class SpeedCoefficient {
    public:
        static constexpr float STOP = 0.0;
        static constexpr float WALKING_PACE = 0.25;
        static constexpr float HALF_SPEED = 0.5;
        static constexpr float NORMAL = 1.0;
    };

    void timer_callback()
    {
        if (will_send_speed_)
        {
            auto message = interfaces::msg::SpeedInfo();
            message.speed_value_front = speed_value_front;
            message.speed_value_back = speed_value_back;
            RCLCPP_INFO(this->get_logger(), "Publishing: [back] '%f' - [front] '%f'", message.speed_value_back, message.speed_value_front);
            publisher_->publish(message);
            will_send_speed_ = false;
        }        
    }


    // is_front = true => front sensor. Else, back sensor
    void update_speed_info(bool is_front, int16_t sensor_value)
    {
        std::string orientation = "[FRONT]";
        if (!is_front)
        {
            orientation = "[BACK]";
        }
        
        // Comparison 
        if ((sensor_value < THRESHOLD_STOP))
        {
            RCLCPP_WARN(this->get_logger(), "'%s' STOP !!!", orientation);
            will_send_speed_ = true;
            if (is_front)
                speed_value_front = SpeedCoefficient::STOP; 
            else
                speed_value_back = SpeedCoefficient::STOP; 
        }
        else if ((sensor_value < THRESHOLD_SLOW))
        {
            RCLCPP_INFO(this->get_logger(), "'%s' SLOW DOWN AGAIN !!!", orientation);
            will_send_speed_ = true;
            if (is_front)
                speed_value_front = SpeedCoefficient::WALKING_PACE; 
            else
                speed_value_back = SpeedCoefficient::WALKING_PACE; 
        }
        else if ((sensor_value < THRESHOLD_FIRST_SLOW))
        {
            RCLCPP_INFO(this->get_logger(), "'%s' SLOW DOWN BOY !!!", orientation);
            will_send_speed_ = true;
            if (is_front)
                speed_value_front = SpeedCoefficient::HALF_SPEED; 
            else
                speed_value_back = SpeedCoefficient::HALF_SPEED; 
        }
        else if ((sensor_value < THRESHOLD_CAREFUL))
        {
            RCLCPP_INFO(this->get_logger(), "'%s' SOMETHING DETECTED, CAREFULL !!!", orientation);
            will_send_speed_ = true;
            if (is_front)
                speed_value_front = SpeedCoefficient::NORMAL; 
            else
                speed_value_back = SpeedCoefficient::NORMAL; 
        }
        else if ((sensor_value > THRESHOLD_CAREFUL))
        {
            // RCLCPP_INFO(this->get_logger(), "CHILL BRO EVERYTHING IS GOOD. NO GRANNY's OTW!!!");
            if (is_front)
                speed_value_front = SpeedCoefficient::NORMAL; 
            else
                speed_value_back = SpeedCoefficient::NORMAL;
        }
    }

    void topic_callback(const interfaces::msg::Ultrasonic::SharedPtr msg)
    {
        int16_t front_center = msg->front_center;
        int16_t rear_center = msg->rear_center;
        will_send_speed_ = false;

        update_speed_info(true, front_center);
        update_speed_info(false, rear_center);
    }

};

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