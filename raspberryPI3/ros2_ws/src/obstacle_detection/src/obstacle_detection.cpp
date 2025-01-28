#include "obstacle_detection.hpp"

ObstacleDetection::ObstacleDetection() 
    : Node("obstacle_detection"), 
        will_send_speed_(false), 
        speed_value_front(SpeedCoefficient::NORMAL_NON_DETECTION), 
        speed_value_back(SpeedCoefficient::NORMAL_NON_DETECTION), 
        parkmod_(false), 
        is_margin_reach_back_(false),
        is_margin_reach_front_(false)
{
    //SUBSCRIBERS
    subscription_ = this->create_subscription<interfaces::msg::Ultrasonic>("/us_data", 10, std::bind(&ObstacleDetection::topic_callback, this, std::placeholders::_1));
    subscription_lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&ObstacleDetection::laserScanCallback, this, std::placeholders::_1));
    subscription_parking_leaving_ = this->create_subscription<std_msgs::msg::Bool>("/parking_leaving_in_process", 10, std::bind(&ObstacleDetection::selectSecurity, this, std::placeholders::_1));

    //PUBLISHERS
    publisher_ = this->create_publisher<interfaces::msg::SpeedInfo>("/speed_info", 10);
    
    timer_ = this->create_wall_timer(50ms, std::bind(&ObstacleDetection::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "obstacle_detection node READY");
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

void ObstacleDetection::selectSecurity(const std_msgs::msg::Bool::SharedPtr parking_leaving){
    /*
        When the car is parking or is leaving a parking space then the Park Security is activated.
    */
    RCLCPP_INFO(this->get_logger(), "PARKMOD : %d", parking_leaving->data);
    set_parkmod(parking_leaving->data);
}

void ObstacleDetection::update_speed_info(
    bool is_front, int16_t sensor_value)
{
    std::string orientation = is_front ? "[FRONT]" : "[BACK]";
  
    if (get_parkmod() == true)
    {
        /*
            PARK SECURITY :
            On the "parkmod", compare ultrasonic sensor with the 15cm margin and 30cm with the lidar. 
            Stop the car if the direction is the same than the ulstrasonic sensor detection
        */
        RCLCPP_DEBUG(this->get_logger(), "parkmod 1 :");
        if (sensor_value < THRESHOLD_PARK_STOP)
        {
            will_send_speed_ = true;

            if (is_front == true) {
                if (speed_value_front != SpeedCoefficient::STOP) {
                    RCLCPP_WARN(this->get_logger(), "'%s' STOP PARK u !!!", orientation.c_str());
                }
                speed_value_front = SpeedCoefficient::STOP; 
            }
            else if (is_front == false) {
                if (speed_value_back != SpeedCoefficient::STOP) {
                    RCLCPP_WARN(this->get_logger(), "'%s' STOP PARK u !!!", orientation.c_str());
                }
                speed_value_back = SpeedCoefficient::STOP; 
            }
        }
        else if (is_margin_reach_back_ == true)
        {
            will_send_speed_ = true;

            if (is_front == false) {
                if (speed_value_back != SpeedCoefficient::STOP) {
                    RCLCPP_WARN(this->get_logger(), "'%s' STOP PARK l back !!!", orientation.c_str());
                }
                speed_value_back = SpeedCoefficient::STOP; 
            }
        }
        else if (is_margin_reach_front_ == true)
        {
            will_send_speed_ = true;

            if (is_front == true) {
                if (speed_value_front != SpeedCoefficient::STOP) {
                    RCLCPP_WARN(this->get_logger(), "'%s' STOP PARK l front !!!", orientation.c_str());
                }
                speed_value_front = SpeedCoefficient::STOP; 
            }
        }
        else
        {
                // No warning or message; speed stays NORMAL.
            will_send_speed_ = true;
            if (is_front)
                speed_value_front = SpeedCoefficient::NORMAL_NON_DETECTION; 
            else
               speed_value_back = SpeedCoefficient::NORMAL_NON_DETECTION;
        }
    }

    /*
        DRIVE SECURITY :
        If the parkmod is false then the margin are evaluate with the ultrasonic sensors
    */
    else if (sensor_value < THRESHOLD_STOP)
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
    
    else if (sensor_value < THRESHOLD_SLOW)
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
    
    else if (sensor_value < THRESHOLD_FIRST_SLOW)
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
    else if (sensor_value < THRESHOLD_CAREFUL)
    {
        if (is_front){
            if (speed_value_front != SpeedCoefficient::NORMAL_DETECTION)
                RCLCPP_INFO(this->get_logger(), "'%s' SOMETHING DETECTED, CAREFULL !!!", orientation.c_str());

            speed_value_front = SpeedCoefficient::NORMAL_DETECTION; 
        } else {
            if (speed_value_back != SpeedCoefficient::NORMAL_DETECTION)
               RCLCPP_INFO(this->get_logger(), "'%s' SOMETHING DETECTED, CAREFULL !!!", orientation.c_str());

            speed_value_back = SpeedCoefficient::NORMAL_DETECTION; 
        }
    }
    else
    {
        // No warning or message; speed stays NORMAL.
        will_send_speed_ = true;
        if (is_front)
            speed_value_front = SpeedCoefficient::NORMAL_NON_DETECTION; 
        else
            speed_value_back = SpeedCoefficient::NORMAL_NON_DETECTION; 
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

/////////////Set the lidar detection//////////////////////


void ObstacleDetection::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::vector<std::pair<float, float>> cartesian_coords; // Pour stocker les coordonnées cartésiennes (x, y)
    constexpr float degree_increment = M_PI / 180.0; // 1° en radians
    constexpr float precision_cm = 0.01; // Précision en mètres (1 cm)

    constexpr float lidar_offset_y = -0.4; // Décalage du Lidar en mètres (excentré de -40 cm)
    // Définir les limites de la zone d'intérêt (en mètres)
    constexpr float rect_min_x = -0.45; // Limite gauche
    constexpr float rect_max_x = 0.45;  // Limite droite
    constexpr float rect_min_y = -0.7; // Limite arrière
    constexpr float rect_max_y = 0.7;  // Limite avant

    size_t num_measurements = static_cast<size_t>((msg->angle_max - msg->angle_min) / degree_increment);
    cartesian_coords.reserve(num_measurements);

    for (size_t i = 0; i < num_measurements; ++i)
    {
        float angle = msg->angle_min + i * degree_increment;
        size_t range_index = static_cast<size_t>((angle - msg->angle_min) / msg->angle_increment);

        if (range_index < msg->ranges.size())
        {
            float range = msg->ranges[range_index];

            if (range > msg->range_min && range < msg->range_max &&
                !std::isinf(range) && !std::isnan(range))
            {
                range = std::round(range / precision_cm) * precision_cm; // Arrondir à 1 cm
                float x = -(range * std::cos(angle));
                float y = -(range * std::sin(angle)) + lidar_offset_y;

                cartesian_coords.emplace_back(x, y);
            }
        }
    }

    // Filtrer les points dans la zone rectangulaire
    std::vector<std::pair<float, float>> filtered_points_front;
    std::vector<std::pair<float, float>> filtered_points_back;

    for (const auto& coord : cartesian_coords)
    {
        if (coord.first >= rect_min_x && coord.first <= rect_max_x &&
            coord.second >= 0 && coord.second <= rect_max_y)
        {
            filtered_points_front.push_back(coord);
        }
        if (coord.first >= rect_min_x && coord.first <= rect_max_x &&
            coord.second >= rect_min_y && coord.second <= 0)
        {
            filtered_points_back.push_back(coord);
        }
    }

    // Detection d'un obstacle
    if (!filtered_points_front.empty())
    {
        if (!is_margin_reach_front_)
        {
            // RCLCPP_WARN(this->get_logger(), "Lidar margin front reached!");
        }
        is_margin_reach_front_ = true;
    }
    else if (filtered_points_front.empty())
    {
        is_margin_reach_front_ = false;
    }
    
    
    if (!filtered_points_back.empty())
    {
        if (!is_margin_reach_back_)
        {
            // RCLCPP_WARN(this->get_logger(), "Lidar margin back reached!");
        }
        is_margin_reach_back_ = true;
    }
    else if (filtered_points_back.empty()) 
    {
        is_margin_reach_back_ = false;
    }
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