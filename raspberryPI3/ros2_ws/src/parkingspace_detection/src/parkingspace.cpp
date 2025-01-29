#include "parkingspace.hpp"
#include <mutex> 

using namespace std::chrono_literals;

ParkingSpace::ParkingSpace() : Node("parking_space"), detected_parking_type_(ParkingType::NONE)
{
	//SUBSCRIBERS
	laser_scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&ParkingSpace::detect_parking_space, this, std::placeholders::_1));
	motors_feedback_subscription_ = this->create_subscription<interfaces::msg::MotorsFeedback>("/motors_feedback", 10, std::bind(&ParkingSpace::increment_parking_space_length, this, std::placeholders::_1));
	subscriber_search_parking = this->create_subscription<std_msgs::msg::Bool>("/start_search", 10, std::bind(&ParkingSpace::init_search, this, std::placeholders::_1));

	//PUBLISHERS
	finish_initialization = this->create_publisher<std_msgs::msg::Bool>("/search_finish_initialization", 10);
	type_place_info = this->create_publisher<std_msgs::msg::Int32>("/info_parking_place", 10);

    //timer_ = this->create_wall_timer(500ms, std::bind(&ParkingSpace::parking_place_info, this));
	
	scan.set_init_compteur(0);
	scan.set_ref_distance_init(0);
	scan.set_ref_distance(0);
	scan.set_isInitialized(true);
	scan.set_isDetecting(false);
	scan.set_place_distance(0);
	
	clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
	
	RCLCPP_INFO(this->get_logger(), "parkingspace_detection node READY");
	
	m_length = 0.0;
	m_depth = 0.0;
	
	RCLCPP_DEBUG(this->get_logger(), "Initialization...");
}

void ParkingSpace::init_search(const std_msgs::msg::Bool::SharedPtr msg){
	search_in_progress = msg->data;

	if(search_in_progress){
		RCLCPP_INFO(this->get_logger(), "Search in progress...");
	} else {
		scan.set_isInitialized(true);
	}
}

void ParkingSpace::detect_parking_space(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
{
	if(search_in_progress){
		RCLCPP_DEBUG(this->get_logger(), "In callback");
	
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

		RCLCPP_DEBUG(this->get_logger(), "Receiving value from callback");

		if (scan.get_isInitialized()){

			if (scan.get_init_compteur()<LIMIT_INIT){
				
				if(!isinf(scan.get_range_at(0))){
					scan.set_ref_distance_init(scan.get_range_at(0));
					scan.set_init_compteur(1);
					RCLCPP_DEBUG(this->get_logger(), "Increasing compteur");
				}
			}
			else{
				scan.set_ref_distance(scan.get_ref_distance_init()/LIMIT_INIT);
				RCLCPP_INFO(this->get_logger(), "Distance of reference set to : %f",scan.get_ref_distance());
				scan.set_isInitialized(false);

				std_msgs::msg::Bool message;
				message.data = true;
				finish_initialization->publish(message);
			}
		}
		else {

			if ((scan.get_range_at(0) >= scan.get_ref_distance() + LENGHT_CAR) && !scan.get_isDetecting()){ // If the distance detected is above or equal to the reference distance plus the lenght of the car
				
				RCLCPP_DEBUG(this->get_logger(), "Beginning of a potential space");
				if (isinf(scan.get_range_at(0))){
					scan.set_place_distance(12.0);
				}
				else{
					scan.set_place_distance(scan.get_range_at(0));
				}
				m_length = 0.0;
				m_depth = 0.0;
				scan.set_isDetecting(true); // We set detected to true as we might have detected a place
			}	
			else if (scan.get_isDetecting() && ((scan.get_range_at(0) >= (scan.get_ref_distance() - LIMIT_DISTANCE)) && (scan.get_range_at(0) <= (scan.get_ref_distance() + LIMIT_DISTANCE)))){ // If we already might have detected a place and we have a distance back in the limit around our ref distance then we might have the end of the place
				RCLCPP_DEBUG(this->get_logger(), "End of a potential space");
				scan.set_isDetecting(false); // We set back the detected to false for new detetction
				hasDetected = true; 
			}
			else if (scan.get_place_distance()==12.0 && scan.get_isDetecting()){
				if (!(isinf(scan.get_range_at(0)))){
					scan.set_place_distance(scan.get_range_at(0));
				}
			}


			if (hasDetected) {
				m_depth = (scan.get_place_distance() - scan.get_ref_distance()) * 100;

				auto message = std_msgs::msg::Int32();

				if (m_length >= PARKING_SPACE_LIMIT_STRAIGHT_LENGTH && m_length <= PARKING_SPACE_LIMIT_PARALLEL_LENGTH-10 && m_depth >= PARKING_SPACE_LIMIT_STRAIGHT_DEPTH) {
					message.data = static_cast<int32_t>(ParkingType::PERPENDICULAR);
					RCLCPP_INFO(this->get_logger(), "Perpendicular parking space detected");
					RCLCPP_INFO(this->get_logger(), "Width: %.2f cm, Depth: %.2f cm", m_length, m_depth);
					
				} else if (m_length >= PARKING_SPACE_LIMIT_PARALLEL_LENGTH && m_depth >= PARKING_SPACE_LIMIT_PARALLEL_DEPTH) {
					message.data = static_cast<int32_t>(ParkingType::PARALLEL);
					RCLCPP_INFO(this->get_logger(), "Parallel parking space detected");
					RCLCPP_INFO(this->get_logger(), "Width: %.2f cm, Depth: %.2f cm", m_length, m_depth);

				} else {
					message.data = static_cast<int32_t>(ParkingType::NONE);
					RCLCPP_DEBUG(this->get_logger(), "Not a valid parking space");
					RCLCPP_DEBUG(this->get_logger(), "Width: %.2f cm, Depth: %.2f cm", m_length, m_depth);
				}

				if(message.data != static_cast<int32_t>(ParkingType::NONE)){
					type_place_info->publish(message);
					search_in_progress = false;
					scan.set_isInitialized(true);
				}
			}
		}
	}
	
}

void ParkingSpace::increment_parking_space_length(const interfaces::msg::MotorsFeedback::SharedPtr msg) {
	if(search_in_progress){
		std::lock(scan_mutex_, length_mutex_);
		std::lock_guard<std::mutex> scan_lock(scan_mutex_, std::adopt_lock);
		std::lock_guard<std::mutex> length_lock(length_mutex_, std::adopt_lock);


		if (scan.get_isDetecting())
		{
			float tmp = msg->left_rear_odometry*WHEEL_DIAMETER*M_PI/PULSE_FOR_A_REVOLUTION;
			m_length += tmp;
			RCLCPP_DEBUG(this->get_logger(), "Added %f", tmp);
		}
	}
}

// void ParkingSpace::parking_place_info(){

// 	if (detected_parking_type_ == ParkingType::NONE) {
//         return; // Do nothing if no valid parking space is detected
//     }

//     auto message = std_msgs::msg::Int32();
//     message.data = static_cast<int32_t>(detected_parking_type_);

//     switch (detected_parking_type_) {
//         case ParkingType::PERPENDICULAR:
//             RCLCPP_INFO(this->get_logger(), "Publishing: Perpendicular parking space");
//             break;
//         case ParkingType::PARALLEL:
//             RCLCPP_INFO(this->get_logger(), "Publishing: Parallel parking space");
//             break;
//         case ParkingType::ANGLED:
//             RCLCPP_INFO(this->get_logger(), "Publishing: Angled parking space");
//             break;
//         default:
//             break;
//     }

//     type_place_info->publish(message);  // Only publish if a valid parking type is detected

// }

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ParkingSpace>());
	rclcpp::shutdown();
	return 0;
}