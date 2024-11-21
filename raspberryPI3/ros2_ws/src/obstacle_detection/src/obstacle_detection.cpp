#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/ultrasonic.hpp"
#include "interfaces/msg/speed_info.hpp"

#define THRESHOLD_STOP 30
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
    float speed_value_;

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
            message.speed_coeff = speed_value_;
            RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.speed_coeff);
            publisher_->publish(message);
            will_send_speed_ = false;
        }        
    }

    void topic_callback(const interfaces::msg::Ultrasonic::SharedPtr msg)
    {
        // Access the fields of the custom message
        //int16_t front_left = msg->front_left;
        int16_t front_center = msg->front_center;
        //int16_t front_right = msg->front_right;
        //int16_t rear_left = msg->rear_left;
        int16_t rear_center = msg->rear_center;
        //int16_t rear_right = msg->rear_right;

        //
        //RCLCPP_INFO(this->get_logger(), "Received Ultrasonic Data");
        //RCLCPP_INFO(this->get_logger(), "Front Left: %d, Front Center: %d, Front Right: %d", front_left, front_center, front_right);
        //RCLCPP_INFO(this->get_logger(), "Rear Left: %d, Rear Center: %d, Rear Right: %d", rear_left, rear_center, rear_right);

        // Comparison 
        if ((front_center < THRESHOLD_STOP) || (rear_center < THRESHOLD_STOP))
        {
            RCLCPP_WARN(this->get_logger(), "STOP !!!");
            will_send_speed_ = true;
            speed_value_ = SpeedCoefficient::STOP; 
        }
        else if ((front_center < THRESHOLD_SLOW) || (rear_center < THRESHOLD_SLOW))
        {
            RCLCPP_INFO(this->get_logger(), "SLOW DOWN AGAIN !!!");
            will_send_speed_ = true;
            speed_value_ = SpeedCoefficient::WALKING_PACE; 
        }
        else if ((front_center < THRESHOLD_FIRST_SLOW) || (rear_center < THRESHOLD_FIRST_SLOW))
        {
            RCLCPP_INFO(this->get_logger(), "SLOW DOWN BOY !!!");
            will_send_speed_ = true;
            speed_value_ = SpeedCoefficient::HALF_SPEED; 
        }
        else if ((front_center < THRESHOLD_CAREFUL) || (rear_center < THRESHOLD_CAREFUL))
        {
            RCLCPP_INFO(this->get_logger(), "SOMETHING DETECTED, CAREFULL !!!");
            will_send_speed_ = true;
            speed_value_ = SpeedCoefficient::NORMAL; 
        }
        else if ((front_center > THRESHOLD_CAREFUL) && (rear_center > THRESHOLD_CAREFUL))
        {
            RCLCPP_INFO(this->get_logger(), "CHILL BRO EVERYTHING IS GOOD. NO GRANNY's OTW!!!");
            will_send_speed_ = false;
            speed_value_ = SpeedCoefficient::NORMAL; 
        }
        // Add further comparisons as needed
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