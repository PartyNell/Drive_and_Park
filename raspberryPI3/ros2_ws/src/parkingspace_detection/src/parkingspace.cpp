#include "parkingspace.hpp"
#include <math.h>


ParkingSpace::ParkingSpace() : Node("parking_space")
{

	subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&ParkingSpace::topic_callback, this, std::placeholders::_1));
	scan.set_init_compteur(0);
	scan.set_ref_distance_init(0);
	scan.set_ref_distance(0);
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
	
	RCLCPP_INFO(this->get_logger(), "Distance ref init de base : %f", scan.get_ref_distance_init());
	RCLCPP_INFO(this->get_logger(), "Distance ref de base : %f", scan.get_ref_distance());

	if (scan.get_init_compteur()<LIMITE_INIT){
		if(isinf(scan.get_range_at(0))){
			//Do nothing
		}else{
			RCLCPP_INFO(this->get_logger(), "Initialization...");
			scan.set_ref_distance_init(scan.get_range_at(0));
			scan.set_init_compteur(1);
		}
	}
	else{
		scan.set_ref_distance(scan.get_ref_distance_init()/LIMITE_INIT);
		RCLCPP_INFO(this->get_logger(), "Done !!!!");
	}

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