#include "parkingspace.hpp"
#include <mutex> 

using namespace std::chrono_literals;

ParkingSpace::ParkingSpace() : Node("parking_space")
{

	laser_scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&ParkingSpace::detect_parking_space, this, std::placeholders::_1));
	motors_feedback_subscription_ = this->create_subscription<interfaces::msg::MotorsFeedback>("/motors_feedback", 10, std::bind(&ParkingSpace::increment_parking_space_length, this, std::placeholders::_1));
	scan.set_init_compteur(0);
	scan.set_ref_distance_init(0);
	scan.set_ref_distance(0);
	scan.set_isInitialized(true);
	scan.set_isDetecting(false);
	scan.set_place_distance(0);
	clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
	RCLCPP_INFO(this->get_logger(), "parkingspace_detection node READY");
	RCLCPP_INFO(this->get_logger(), "Initialization...");
	m_length = 0.0;
	m_depth = 0.0;
}

void ParkingSpace::detect_parking_space(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
{
	std::lock(scan_mutex_, length_mutex_);
	std::lock_guard<std::mutex> scan_lock(scan_mutex_, std::adopt_lock); 
    std::lock_guard<std::mutex> length_lock(length_mutex_, std::adopt_lock); 

	scan.set_angle_min(msg->angle_min);
	scan.set_angle_max(msg->angle_max);
	scan.set_angle_increment(msg->angle_increment);
	scan.set_time_increment(msg->time_increment);
	scan.set_scan_time(msg->scan_time);
	scan.set_range_min(msg->range_min);
	scan.set_range_max(msg->range_max);
	scan.set_ranges(msg->ranges.data());
	scan.set_intensities(msg->intensities.data());
	bool hasDetected = false;

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

			if ((scan.get_range_at(0) >= scan.get_ref_distance() + LENGHT_CAR) && !scan.get_isDetecting()){ // If the distance detected is above or equal to the reference distance plus the lenght of the car
				event_1_time_ = clock_->now(); // Then we capture the current time and print it for info purpose
				RCLCPP_INFO(this->get_logger(), "Beginning of a potential place");
				scan.set_place_distance(scan.get_range_at(0));
				m_length = 0.0;
				m_depth = 0.0;
				scan.set_isDetecting(true); // We set detected to true as we might have detected a place
			}
		
			else if (scan.get_isDetecting() && ((scan.get_range_at(0) >= (scan.get_ref_distance() - LIMIT_DISTANCE)) && (scan.get_range_at(0) <= (scan.get_ref_distance() + LIMIT_DISTANCE)))){ // If we already might have detected a place and we have a distance back in the limit around our ref distance then we might have the end of the place
				
				event_2_time_ = clock_->now();  // So we capture the current time for the second time
				RCLCPP_INFO(this->get_logger(), "I might have detected a parking space");
				scan.set_isDetecting(false); // We set back the detected to false for new detetction
				hasDetected = true; 
			}
		}
		else{

			// RCLCPP_INFO(this->get_logger(), "Distance Inf !!!!");
		}

	if (hasDetected){
		// Compute both distance to get the size or type of a place
		m_depth = (scan.get_place_distance()-scan.get_ref_distance()) * 100;	// in cm
		RCLCPP_INFO(this->get_logger(), "Depth of the place  %f", m_depth);
		RCLCPP_INFO(this->get_logger(), "Width of the place  %f", m_length);

		// compute the full distance 
		if (	m_length >= PARKING_SPACE_LIMIT_STRAIGHT_LENGTH 
			&& 	m_depth >= PARKING_SPACE_LIMIT_STRAIGHT_DEPTH)
		{
			RCLCPP_INFO(this->get_logger(), "Straight Parking space detected");
		}
		else if (	m_length >= PARKING_SPACE_LIMIT_PARALLEL_LENGTH 
				&& 	m_depth >= PARKING_SPACE_LIMIT_PARALLEL_DEPTH)
		{
			RCLCPP_INFO(this->get_logger(), "Parallel Parking space detected");
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "Not a parking space, keep going !");
		}
	}
	

	//float dist_at_right = scan.get_range_at(0);
	//RCLCPP_INFO(this->get_logger(), "Distance à droite : %f", dist_at_right);
	}
	
}

void ParkingSpace::increment_parking_space_length(const interfaces::msg::MotorsFeedback::SharedPtr msg) {
	
    std::lock(scan_mutex_, length_mutex_);
    std::lock_guard<std::mutex> scan_lock(scan_mutex_, std::adopt_lock);
    std::lock_guard<std::mutex> length_lock(length_mutex_, std::adopt_lock);


	if (scan.get_isDetecting())
	{
		float tmp = msg->left_rear_odometry*WHEEL_DIAMETER*M_PI/PULSE_FOR_A_REVOLUTION;
		m_length += tmp;
		RCLCPP_INFO(this->get_logger(), "Added %f", tmp);
	}
}


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ParkingSpace>());
	rclcpp::shutdown();
	return 0;
}