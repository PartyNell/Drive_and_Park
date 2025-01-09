#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/joystick_order.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class AutonomousDriving : public rclcpp::Node
{
  public:
    AutonomousDriving()
    : Node("autonomous_driving"), count_(0)
    {
      publisher_car_order_ = this->create_publisher<interfaces::msg::JoystickOrder>("autonomous_car_order", 10);
      //publisher_init_state_ = this->create_publisher<std_msgs::msg::Bool>("start_init", 10);
      //publisher_parking_state_ = this->create_publisher<std_msgs::msg::Bool>("start_parking", 10);
      publisher_leaving_state_ = this->create_publisher<std_msgs::msg::Bool>("start_leaving", 10);

      //subscriber_autonomous_mode_ = this->create_subscription<interfaces::msg::JoystickOrder>("joystick_order", 10, std::bind(&AutonomousDriving::parking, this, _1));
      subscriber_autonomous_mode_leaving_ = this->create_subscription<interfaces::msg::JoystickOrder>("joystick_order", 10, std::bind(&AutonomousDriving::leaving, this, _1));
      //subscriber_init_ok_ = this->create_subscription<std_msgs::msg::Bool>("init_finished", 10, std::bind(&AutonomousDriving::start_straight, this, _1));
      //subscriber_parking_ok_ = this->create_subscription<std_msgs::msg::Bool>("parking_finished", 10, std::bind(&AutonomousDriving::wait_order, this, _1));
      subscriber_leaving_ok_ = this->create_subscription<std_msgs::msg::Bool>("leaving_finished", 10, std::bind(&AutonomousDriving::wait_order_leaving, this, _1));
      timer_ = this->create_wall_timer(50ms, std::bind(&AutonomousDriving::timer_callback, this));

      RCLCPP_INFO(this->get_logger(), "autonomous_driving node READY");
    }

  private:
    // void init_autonomous_mode(const interfaces::msg::JoystickOrder & joystick){
    //   if(mode != joystick.mode){
    //     mode = joystick.mode;

    //     if(mode == 1){
    //         std_msgs::msg::Bool init_autonomous;
    //         init_autonomous.data = true;
    //         publisher_init_state_->publish(init_autonomous);

    //         init_in_progress = true;
    //         search_in_progress = false;
    //     } else if (mode != 1 && init_in_progress){
    //         std_msgs::msg::Bool init_autonomous;
    //         init_autonomous.data = false; 
    //         publisher_init_state_->publish(init_autonomous);

    //         init_in_progress = false;
    //         search_in_progress = false;
    //     }
    //   }
    // }

    // void start_straight(const std_msgs::msg::Bool & init){
    //     if(init.data){
    //       RCLCPP_INFO(this->get_logger(), "Initialisation DONE");

    //       car_order.start = true;
    //       car_order.mode = 1;
    //       car_order.throttle = 0.7;
    //       car_order.steer = 0.0;
    //       car_order.reverse = false;

    //       search_in_progress = true;

    //     } else {
    //       RCLCPP_INFO(this->get_logger(), "Initialisation FAILED");

    //       //STOP the car
    //       car_order.start = true;
    //       car_order.mode = 1;
    //       car_order.throttle = 0.0;
    //       car_order.steer = 0.0;
    //       car_order.reverse = false;

    //       search_in_progress = false;
    //     }
    // }

    /*
    void parking (const interfaces::msg::JoystickOrder & joystick)
    {
      if(mode != joystick.mode){
        mode = joystick.mode;

        if(mode == 1){
            std_msgs::msg::Bool init_parking;
            init_parking.data = true;
            publisher_parking_state_->publish(init_parking);

            init_in_progress = false;
            search_in_progress = false;
            parking_in_progress = true;
            
        } else if (mode != 1 && parking_in_progress){
            std_msgs::msg::Bool init_parking;
            init_parking.data = false; 
            publisher_parking_state_->publish(init_parking);

            init_in_progress = false;
            search_in_progress = false;
            parking_in_progress = false;
        }
      }
    }
  */

    void leaving (const interfaces::msg::JoystickOrder & joystick)
    {
      if(mode != joystick.mode){
        mode = joystick.mode;

        if(mode == 1){
            std_msgs::msg::Bool init_leaving;
            init_leaving.data = true;
            publisher_leaving_state_->publish(init_leaving);

            init_in_progress = false;
            search_in_progress = false;
            leaving_in_progress = true;
            
        } else if (mode != 1 && leaving_in_progress){
            std_msgs::msg::Bool init_leaving;
            init_leaving.data = false; 
            publisher_leaving_state_->publish(init_leaving);

            init_in_progress = false;
            search_in_progress = false;
            leaving_in_progress = false;
        }
      }
    }

    void wait_order(const std_msgs::msg::Bool & parking)
    {
      if(parking.data){
        RCLCPP_INFO(this->get_logger(), "Parking SUCCEED");
      } else {
        RCLCPP_INFO(this->get_logger(), "Parking FAILED");
      }

    }

    void wait_order_leaving(const std_msgs::msg::Bool & leaving)
    {
      if(leaving.data){
        RCLCPP_INFO(this->get_logger(), "Leaving Parking Space SUCCEED");
      } else {
        RCLCPP_INFO(this->get_logger(), "Leaving Parking SpaceFAILED");
      }
    }

    void timer_callback()
    {
      if(search_in_progress)
        publisher_car_order_->publish(car_order);
    }

    

    rclcpp::TimerBase::SharedPtr timer_;

    //Publishers
    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr publisher_car_order_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_init_state_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_parking_state_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_leaving_state_;


    //Subscribers
    rclcpp::Subscription<interfaces::msg::JoystickOrder>::SharedPtr subscriber_autonomous_mode_;
    rclcpp::Subscription<interfaces::msg::JoystickOrder>::SharedPtr subscriber_autonomous_mode_leaving_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_init_ok_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_parking_ok_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_leaving_ok_;
    
    //Attributes
    size_t count_;
    int mode = 0;
    bool init_in_progress = false;
    bool search_in_progress = false;
    bool parking_in_progress = false;
    bool leaving_in_progress = false;
    interfaces::msg::JoystickOrder car_order;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutonomousDriving>());
  rclcpp::shutdown();
  return 0;
}