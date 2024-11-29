#include "parkingspace.hpp"
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <iostream>
#include <ctime>

using namespace std::chrono_literals;

ParkingSpace::ParkingSpace() : Node("parking_space")
{

	subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&ParkingSpace::topic_callback, this, std::placeholders::_1));
	scan.set_init_compteur(0);
	scan.set_ref_distance_init(0);
	scan.set_ref_distance(0);
	scan.set_isInitialized(true);
	scan.set_isDetected(false);
	scan.set_place_distance(0);
	clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
	RCLCPP_INFO(this->get_logger(), "parkingspace_detection node READY");
	RCLCPP_INFO(this->get_logger(), "Initialization...");
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
	bool calcul_place_dimension = false;

	if (scan.get_isInitialized()){

		if (scan.get_init_compteur()<LIMIT_INIT){
			
			if(!isinf(scan.get_range_at(0))){
				scan.set_ref_distance_init(scan.get_range_at(0));
				scan.set_init_compteur(1);
			}
		}
		else{
			scan.set_ref_distance(scan.get_ref_distance_init()/LIMIT_INIT);
			RCLCPP_INFO(this->get_logger(), "Done !!!!");
			RCLCPP_INFO(this->get_logger(), "Distance of reference set to : %f",scan.get_ref_distance());
			scan.set_isInitialized(false);
		}
	}
	else {
		
		if (!isinf(scan.get_range_at(0))){

			if ((scan.get_range_at(0) >= scan.get_ref_distance() + LENGHT_CAR) && !scan.get_isDetected()){ // If the distance detected is above or equal to the reference distance plus the lenght of the car
				
				event_1_time_ = clock_->now(); // Then we capture the current time and print it for info purpose
				RCLCPP_INFO(this->get_logger(), "Detected difference of distance at: %f.%09ld", event_1_time_.seconds(), event_1_time_.nanoseconds());
				scan.set_place_distance(scan.get_range_at(0));
				scan.set_isDetected(true); // We set detected to true as we might have detected a place
			}
		
			if (scan.get_isDetected() && ((scan.get_range_at(0) >= (scan.get_ref_distance() - LIMIT_DISTANCE)) || (scan.get_range_at(0) <= (scan.get_ref_distance() + LIMIT_DISTANCE)))){ // If we already might have detected a place and we have a distance back in the limit around our ref distance then we might have the end of the place
				
				event_2_time_ = clock_->now();  // So we capture the current time for the second time
				RCLCPP_INFO(this->get_logger(), "Going back to the reference distance: %f.%09ld", event_2_time_.seconds(), event_2_time_.nanoseconds());
				scan.set_isDetected(false); // We set back the detected to false for new detetction
				calcul_place_dimension = true; 
			}
		}
		else{

			RCLCPP_INFO(this->get_logger(), "Distance Inf !!!!");
		}

	if (calcul_place_dimension){
		// Compute the time difference between event 1 and event 2
    	rclcpp::Duration time_difference = event_2_time_ - event_1_time_;
		RCLCPP_INFO(this->get_logger(), "Time difference between Event 1 and Event 2: %f seconds", time_difference.seconds());

		// Compute both distance to get the size or type of a place
		RCLCPP_INFO(this->get_logger(), "Witdh of the place  %f", (scan.get_place_distance()-LENGHT_CAR));
	}
	

	//float dist_at_right = scan.get_range_at(0);
	//RCLCPP_INFO(this->get_logger(), "Distance à droite : %f", dist_at_right);
	}
	
}


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ParkingSpace>());
	rclcpp::shutdown();
	return 0;
}