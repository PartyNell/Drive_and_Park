#include "obstacle_detection.hpp"

ObstacleDetection::ObstacleDetection() 
    : Node("obstacle_detection"), 
        will_send_speed_(false), 
        speed_value_front(SpeedCoefficient::NORMAL), 
        speed_value_back(SpeedCoefficient::NORMAL), 
        parkmod_(true), 
        is_margin_reach_back_(false),
        is_margin_reach_front_(false)
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
    On the "parkmod", compare ultrasonic sensor with the 15cm margin and 30cm with the lidar. 
    Stop the car if the direction is the same than the ulstrasonic sensor detection
    */
    if (get_parkmod() == true)
    {
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
                    RCLCPP_WARN(this->get_logger(), "'%s' STOP PARK l !!!", orientation.c_str());
                }
                speed_value_back = SpeedCoefficient::STOP; 
            }
        }
        else if (is_margin_reach_front_ == true)
        {
            will_send_speed_ = true;

            if (is_front == true) {
                if (speed_value_front != SpeedCoefficient::STOP) {
                    RCLCPP_WARN(this->get_logger(), "'%s' STOP PARK l !!!", orientation.c_str());
                }
                speed_value_front = SpeedCoefficient::STOP; 
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

////////////////////////////////////////////////////////////////////////////////
     
    else if (get_parkmod() == false &&
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
    else if (get_parkmod() == false)
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

/////////////Set the lidar detection//////////////////////
/* 
*/
void ObstacleDetection::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    bool margin_reached_back = false;
    bool margin_reached_front = false;

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    
    std::vector<std::pair<float, float>> cartesian_coords; // Pour stocker les coordonnées cartésiennes (x, y)
    constexpr float degree_increment = M_PI / 180.0; // 1° en radians
    constexpr float precision_cm = 0.01; // Précision en mètres (1 cm)

    constexpr float lidar_offset_y = -0.4; // Décalage du Lidar en mètres (excentré de -40 cm)
    // Définir les limites de la zone d'intérêt (en mètres)
    constexpr float rect_min_x = -2.0; // Limite gauche
    constexpr float rect_max_x = 2.0;  // Limite droite
    constexpr float rect_min_y = -1.0; // Limite arrière
    constexpr float rect_max_y = 3.0;  // Limite avant

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
                float x = range * std::cos(angle);
                float y = range * std::sin(angle) + lidar_offset_y;

                cartesian_coords.emplace_back(x, y);
            }
        }
    }

    // Affichage des coordonnées cartésiennes
    std::ostringstream oss;
    oss << "Cartesian coordinates (x, y):\n";
    for (const auto& coord : cartesian_coords)
    {
        oss << "(" << coord.first << ", " << coord.second << ")\n";
    }
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

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
            RCLCPP_WARN(this->get_logger(), "Lidar margin front reached!");
        }
        is_margin_reach_front_ = true;
    }
    else if (!filtered_points_back.empty())
    {
        if (!is_margin_reach_back_)
        {
            RCLCPP_WARN(this->get_logger(), "Lidar margin back reached!");
        }
        is_margin_reach_back_ = true;
    }
    else
    {
        is_margin_reach_back_ = false;
        is_margin_reach_front_ = false;
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