#include "parkingspace.hpp"

class ParkingSpace : public rclcpp::Node
{
  public:
    ParkingSpace() : Node("parking_space")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "topic", 10, std::bind(&ParkingSpace::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
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