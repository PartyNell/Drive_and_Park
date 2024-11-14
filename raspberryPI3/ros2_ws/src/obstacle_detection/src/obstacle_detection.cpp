#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/Speed_info.hpp"

class ObstacleDetection : public rclcpp::Node
{
public:
    ObstacleDetection() : Node("obstacle_detection")
    {
        // Create a subscription to the "/us_data" topic
        subscription_ = this->create_subscription<interfaces::msg::Ultrasonic>("/us_data", 10, std::bind(&ObstacleDetection::topic_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<interfaces::msg::Speed_info>("Speed_info", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&ObstacleDetection::timer_callback, this));
    }

private:

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;

    rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr subscription_;
 
    bool will_send_speed_;
    float speed_value_;

    enum Speed_coefficient = {
        WALKING_PACE = 0.25,
        HALF_SPEED = 0.5,
        SLOWER = 0.75,
        NORMAL = 1.0
    };


    void timer_callback()
    {
        if (will_send_speed_)
        {
            auto message = interfaces::msg::Speed_info();
            message.speed_coeff = speed_value_;
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.speed_coeff.c_str());
            publisher_->publish(message);
            will_send_speed_ = false;
        }        
    }

    void topic_callback(const interfaces::msg::Ultrasonic::SharedPtr msg)
    {
        // Access the fields of the custom message
        int16_t front_left = msg->front_left;
        int16_t front_center = msg->front_center;
        int16_t front_right = msg->front_right;
        int16_t rear_left = msg->rear_left;
        int16_t rear_center = msg->rear_center;
        int16_t rear_right = msg->rear_right;

        //
        RCLCPP_INFO(this->get_logger(), "Received Ultrasonic Data");
        //RCLCPP_INFO(this->get_logger(), "Front Left: %d, Front Center: %d, Front Right: %d", front_left, front_center, front_right);
        //RCLCPP_INFO(this->get_logger(), "Rear Left: %d, Rear Center: %d, Rear Right: %d", rear_left, rear_center, rear_right);

        // Comparison 
        if ((front_left < 30) || (front_center < 30) || (front_right < 30) || (rear_left < 30) || (rear_center < 30) || (rear_right < 30))
        {
            RCLCPP_WARN(this->get_logger(), "STOP !!!");
            will_send_speed_ = true;
            speed_value_ = Speed_coefficient.WALKING_PACE; 
        }
        else if ((front_left < 45) || (front_center < 45) || (front_right < 45) || (rear_left < 45) || (rear_center < 45) || (rear_right < 45))
        {
            RCLCPP_INFO(this->get_logger(), "SLOW DOWN AGAIN !!!");
            will_send_speed_ = true;
            speed_value_ = Speed_coefficient.HALF_SPEED; 
        }
        else if ((front_left < 70) || (front_center < 70) || (front_right < 70) || (rear_left < 70) || (rear_center < 70) || (rear_right < 70))
        {
            RCLCPP_INFO(this->get_logger(), "SLOW DOWN BOY");
            will_send_speed_ = true;
            speed_value_ = Speed_coefficient.SLOWER; 
        }
        else if ((front_left > 100) && (front_center > 100) && (front_right > 100) && (rear_left > 100) && (rear_center > 100) && (rear_right > 100))
        {
            RCLCPP_INFO(this->get_logger(), "EVERYTHING IS GOOD");
            will_send_speed_ = true;
            speed_value_ = Speed_coefficient.NORMAL; 
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