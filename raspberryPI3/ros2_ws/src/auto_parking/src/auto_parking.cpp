#include "auto_parking.hpp"

AutoParking::AutoParking() 
    : Node("auto_parking")
{
    timer_ = this->create_wall_timer(50ms, std::bind(&AutoParking::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "auto_parking node READY");
}

void AutoParking::timer_callback()
{

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
