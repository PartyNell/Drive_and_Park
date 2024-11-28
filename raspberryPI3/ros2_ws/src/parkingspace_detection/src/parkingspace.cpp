#include "parkingspace.hpp"

class ParkingSpace : public rclcpp::Node
{
  public:
    ParkingSpace() : Node("parking_space")
    {
		subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"topic", 10, std::bind(&ParkingSpace::topic_callback, this, _1));
		scan.ranges = new float[scan.nb_points];
		scan.intensities = new float[scan.nb_points];
		scan.nb_points = NB_POINTS; 
    }

    ~ParkingSpace()
    {
		delete[] scan.ranges;
		delete[] scan.intensities;
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
		scan.angle_min = msg->angle_min;
		scan.angle_max = msg->angle_max;
		scan.angle_increment = msg->angle_increment;
		scan.time_increment = msg->time_increment;
		scan.scan_time = msg->scan_time;
		scan.range_min = msg->range_min;
		scan.range_max = msg->range_max; 
		for (int i = 0; i < scan.nb_points; i++)
		{
			scan.ranges[i] = msg->ranges[i];
		}
		
		for (int i = 0; i < scan.nb_points; i++)
		{
			scan.intensities[i] = msg->intensities[i];
		}
		RCLCPP_INFO(this->get_logger(), "I got the data");
      
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ParkingSpace>());
	rclcpp::shutdown();
	return 0;
}