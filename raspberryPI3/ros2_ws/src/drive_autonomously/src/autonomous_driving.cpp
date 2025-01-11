#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/joystick_order.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

#define PULSE_FOR_A_REVOLUTION 36
#define WHEEL_DIAMETER 20.0
#define MAX_DISTANCE 500 //in centimeters

using namespace std::chrono_literals;
using std::placeholders::_1;

class AutonomousDriving : public rclcpp::Node
{
  public:
    AutonomousDriving()
    : Node("autonomous_driving"), count_(0)
    {
      //PUBLISHERS
      publisher_car_order_ = this->create_publisher<interfaces::msg::JoystickOrder>("autonomous_car_order", 10);

      //STATES publishers
      publisher_init_state_ = this->create_publisher<std_msgs::msg::Bool>("start_init", 10);
      publisher_search_parking = this->create_publisher<std_msgs::msg::Bool>("/start_search", 10);

      //SUBSCRIBERS
      subscriber_autonomous_mode_ = this->create_subscription<interfaces::msg::JoystickOrder>("joystick_order", 10, std::bind(&AutonomousDriving::init_autonomous_mode, this, _1));
      subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>("motors_feedback", 10, std::bind(&AutonomousDriving::computeDistance, this, _1));
      subscriber_detect_parking = this->create_subscription<std_msgs::msg::Int32>("/info_parking_place", 10, std::bind(&AutonomousDriving::detect_parking, this, _1));

      //STATES subscribers
      subscriber_init_ok_ = this->create_subscription<std_msgs::msg::Bool>("init_finished", 10, std::bind(&AutonomousDriving::init_search_state, this, _1));

      timer_ = this->create_wall_timer(50ms, std::bind(&AutonomousDriving::timer_callback, this));

      RCLCPP_INFO(this->get_logger(), "autonomous_driving node READY");  
    }

  private:
    void init_autonomous_mode(const interfaces::msg::JoystickOrder & joystick){
      /*
        Initialization of the Autonomous driving mode. Should be executed if the mode switched to Autonomous
      */

      if(mode != joystick.mode){
        mode = joystick.mode;

        if(mode == 1){
          //START initialization
          std_msgs::msg::Bool init_autonomous;
          init_autonomous.data = true;
          publisher_init_state_->publish(init_autonomous);

          init_in_progress = true;
          search_in_progress = false;
          manual = false;

        } else if (mode != 1){
          if(init_in_progress){
            //STOP initialization
            std_msgs::msg::Bool init_autonomous;
            init_autonomous.data = false; 
            publisher_init_state_->publish(init_autonomous);
          }
          else if(search_in_progress){
            //STOP searching empty parking space
            std_msgs::msg::Bool search_parking;
            search_parking.data = false; 
            publisher_search_parking->publish(search_parking);
          }
            init_in_progress = false;
            search_in_progress = false;
            manual = true;
        }
      }
    }

    void init_search_state(const std_msgs::msg::Bool & init){
      /*
        If the execution of the initilization succeed then the car switch to the search state. 3 actions are done at the same time :  
          - the car should drive straighlty
          - the car should compute the traveled distance from the beginning of the search state
          - the car looking for an empty parking space
      */
        init_in_progress = false;

        if(init.data){
          RCLCPP_INFO(this->get_logger(), "Initialisation DONE");

          set_car_order(true, 1, 0.7, 0.0, false);
          distance_travelled = 0.0;

          //start looking at an empty parking space
          std_msgs::msg::Bool search_parking;
          search_parking.data = true; 
          publisher_search_parking->publish(search_parking);

          search_in_progress = true;

        } else {
          RCLCPP_INFO(this->get_logger(), "Initialisation FAILED");

          //STOP the car and switch to manual mode
          set_car_order(true, 0, 0.0, 0.0, false);

          search_in_progress = false;
        }
    }


    void computeDistance(const interfaces::msg::MotorsFeedback & motorsFeedback)
    {
      if (search_in_progress && mode == 1)
      {
        distance_to_add = motorsFeedback.left_rear_odometry*WHEEL_DIAMETER*M_PI/PULSE_FOR_A_REVOLUTION;

        // Compute the distance travelled from the beginning
        distance_travelled += distance_to_add; 

        // If the distance travelled has reached the maximum distance required,
        if (distance_travelled >= MAX_DISTANCE)
        {
          if(!manual){
            // The speed sent to the control of the car is 0
            set_car_order(true, 0, 0.0, 0.0, false);
            RCLCPP_INFO(this->get_logger(), "The car has driven %.2f centimeters.", distance_travelled);
            manual = true;
          }
        }
      }
    }

    void timer_callback()
    {
      if(search_in_progress)
        publisher_car_order_->publish(car_order);
    }

    void detect_parking(const std_msgs::msg::Int32 & space_detected){
      /*
        If a place as been detected then the protocol is throw
          - Straight parking protocol
          - Parallel parking protocol
      */
      parking_type = space_detected.data;
    }


    //TOOLS
    void set_car_order(bool start, int mode, float throttle, float steer, bool reverse){
      car_order.start = start;
      car_order.mode = mode;
      car_order.throttle = throttle;
      car_order.steer = steer;
      car_order.reverse = reverse;
    }

    

    rclcpp::TimerBase::SharedPtr timer_;

    //Publishers
    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr publisher_car_order_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_init_state_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_search_parking;

    rclcpp::Subscription<interfaces::msg::JoystickOrder>::SharedPtr subscriber_autonomous_mode_;
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_init_ok_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_detect_parking;
    
    //Attributes
    size_t count_;
    int mode = 0;
    bool init_in_progress = false;
    bool search_in_progress = false;
    bool manual = false;
    interfaces::msg::JoystickOrder car_order;

	  float distance_to_add = 0.0; 
    float distance_travelled = 0.0;

    int parking_type;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutonomousDriving>());
  rclcpp::shutdown();
  return 0;
}