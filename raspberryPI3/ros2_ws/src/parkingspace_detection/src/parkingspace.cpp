#include "parkingspace.hpp"


ParkingSpace::ParkingSpace() : Node("parking_space")
{
  RCLCPP_INFO(this->get_logger(), "I'm Here !!!!!");
	subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&ParkingSpace::topic_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "parkingspace_detection node READY");
}

void ParkingSpace::topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
{
	scan.set_angle_min(msg->angle_min);
	scan.set_angle_max(msg->angle_max);
	scan.set_angle_increment(msg->angle_increment);
	scan.set_time_increment(msg->time_increment);
	scan.set_scan_time(msg->scan_time);
	scan.set_range_min(msg->range_min);
	scan.set_range_max(msg->range_max);
	scan.set_ranges(msg->ranges.data());
	scan.set_intensities(msg->intensities.data());

	// RCLCPP_INFO(this->get_logger(), "I got the data");
	// RCLCPP_INFO(this->get_logger(), "La valeur min est à l'indice %d",scan.rechercherMin(scan.get_ranges()));
	float dist_at_right = scan.get_range_at(0);
	RCLCPP_INFO(this->get_logger(), "Distance à droite : %f", dist_at_right);
	
}



int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ParkingSpace>());
	rclcpp::shutdown();
	return 0;
}