#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/joystick_order.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

#include "../../parkingspace_detection/include/variable.hpp"

#define PULSE_FOR_A_REVOLUTION 36
#define WHEEL_DIAMETER 20.0
#define MAX_DISTANCE 350 //in centimeters


#define MANUAL 0
#define SEARCH_IN_PROGRESS 1
#define PARKING_IN_PROGRESS 2
#define PARKED 3
#define LEAVING_IN_PROGRESS 4

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
      publisher_park_leave_ = this->create_publisher<std_msgs::msg::Bool>("/parking_leaving_in_process", 10);

      //STATES publishers
      publisher_init_state_ = this->create_publisher<std_msgs::msg::Bool>("start_init", 10);
      publisher_search_parking = this->create_publisher<std_msgs::msg::Bool>("/start_search", 10);
      publisher_parking_state_ = this->create_publisher<std_msgs::msg::Int32>("start_parking", 10);
      publisher_leaving_state_ = this->create_publisher<std_msgs::msg::Int32>("start_leaving", 10);

      //SUBSCRIBERS
      subscriber_autonomous_mode_ = this->create_subscription<interfaces::msg::JoystickOrder>("joystick_order", 10, std::bind(&AutonomousDriving::init_autonomous_mode, this, std::placeholders::_1));
      subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>("motors_feedback", 10, std::bind(&AutonomousDriving::computeDistance, this, _1));

      //STATES subscribers
      subscriber_start_detection = this->create_subscription<std_msgs::msg::Bool>("/search_finish_initialization", 10, std::bind(&AutonomousDriving::init_search_state, this, _1));
      subscriber_detect_parking = this->create_subscription<std_msgs::msg::Int32>("/info_parking_place", 10, std::bind(&AutonomousDriving::detect_parking, this, _1));
      subscriber_parking_ok_ = this->create_subscription<std_msgs::msg::Bool>("parking_finished", 10, std::bind(&AutonomousDriving::wait_order, this, _1));
      subscriber_leaving_ok_ = this->create_subscription<std_msgs::msg::Bool>("leaving_finished", 10, std::bind(&AutonomousDriving::finish_, this, _1));

      RCLCPP_INFO(this->get_logger(), "autonomous_driving node READY");  
    }

  private:

    void init_autonomous_mode(const interfaces::msg::JoystickOrder & joystick){
      /*
        Initialization of the Autonomous driving mode. Should be executed if the mode switched to Autonomous.
        If the car is parked then it launch the leave parking function
      */
      if(mode != joystick.mode){
        mode = joystick.mode;

        if(mode == 1){
          if(manual){
            //START initialization

            RCLCPP_INFO(this->get_logger(), "START LiDar Initialization");

            set_car_order(true, 1, 0.0, 0.0, false);

            //stop the car for LiDar initialization
            std_msgs::msg::Bool init_LiDar;
            init_LiDar.data = true; 
            publisher_search_parking->publish(init_LiDar);

            set_state(SEARCH_IN_PROGRESS);

          } else if (parked) {
            //START leave parking protocole
            std_msgs::msg::Int32 init_leaving;
            init_leaving.data = parking_type;
            publisher_leaving_state_->publish(init_leaving);

            set_parking_security_mode(true);

            set_state(LEAVING_IN_PROGRESS);
          }

        } else if (mode != 1){
          if(search_in_progress){
            //STOP searching empty parking space
            std_msgs::msg::Bool search_parking;
            search_parking.data = false; 
            publisher_search_parking->publish(search_parking);

            set_state(MANUAL);
          } 
          else if(parking_in_progress){
            //STOP the parking state machine
            std_msgs::msg::Int32 parking;
            parking.data = -1; 
            publisher_parking_state_->publish(parking);

            set_state(MANUAL);
          }
          else if(leaving_in_progress){
            //STOP the leaving state machine
            std_msgs::msg::Int32 init_leaving;
            init_leaving.data = -1;
            publisher_leaving_state_->publish(init_leaving);

            set_state(MANUAL);
          }
          else if (parked) {
            set_state(PARKED);
          }
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
        if(init.data){
          RCLCPP_INFO(this->get_logger(), "LiDar Initialisation DONE");

          set_car_order(true, 1, 0.5, 0.0, false);
          distance_travelled = 0.0;

          set_state(SEARCH_IN_PROGRESS);
        } else {
          RCLCPP_INFO(this->get_logger(), "LiDar Initialisation FAILED");

          //STOP the car and switch to manual mode
          set_car_order(true, 1, 0.0, 0.0, false);

          search_in_progress = false;
          set_state(MANUAL);
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
            set_car_order(true, 1, 0.0, 0.0, false);
            RCLCPP_INFO(this->get_logger(), "The car has driven %.2f centimeters.", distance_travelled);
            set_state(MANUAL);
          }
        }
      }

    }

    void detect_parking(const std_msgs::msg::Int32 & space_detected){
      /*
        If a place as been detected then the protocol is throw
          - Straight parking protocol
          - Parallel parking protocol
      */
      parking_type = space_detected.data;

      RCLCPP_DEBUG(this->get_logger(), "PARKING TYPE : %d", parking_type);

      //STOP the car and wait 10 seconds
      set_car_order(true, 1, 0.0, 0.0, false);

      rclcpp::sleep_for(std::chrono::seconds(5));

      //START parking operation
      set_state(PARKING_IN_PROGRESS);

      std_msgs::msg::Int32 parking;
      parking.data = parking_type; 
      publisher_parking_state_->publish(parking);
    }

    void wait_order(const std_msgs::msg::Bool parked){
      /*
        Once the car parked it switch to manual mode and stop the car
      */
      set_state(PARKED);
      set_car_order(true, 1, 0.0, 0.0, false);
    }

    void finish_(const std_msgs::msg::Bool leaved){
      /*
        Once the car leaved the parking space the car stop and switch to manual mode
      */
      //STOP the car
      set_car_order(true, 1, 0.0, 0.0, false);

      set_state(MANUAL);
    }

    //TOOLS
    void set_car_order(bool start, int mode, float throttle, float steer, bool reverse){
      car_order.start = start;
      car_order.mode = mode;
      car_order.throttle = throttle;
      car_order.steer = steer;
      car_order.reverse = reverse;

      publisher_car_order_->publish(car_order);
    }

    void set_parking_security_mode(bool mode_parking){
      std_msgs::msg::Bool security_mode;
      security_mode.data = mode_parking; 
      publisher_park_leave_->publish(security_mode);
    }

    void set_state(int state){
      search_in_progress = false;
      parking_in_progress = false;
      parked = false;
      leaving_in_progress = false;
      manual = false;

      switch(state){
        case 0:
          manual = true;
          set_parking_security_mode(false);
        break;
        case 1:
          search_in_progress = true;
          set_parking_security_mode(false);
        break;
        case 2:
          parking_in_progress = true;
          set_parking_security_mode(true);
        break;
        case 3:
          parked = true;
          set_parking_security_mode(false);
        break;
        case 4:
          leaving_in_progress = true;
          set_parking_security_mode(true);
        break;

      }
    }



    //Publishers
    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr publisher_car_order_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_park_leave_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_init_state_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_parking_state_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_leaving_state_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_search_parking;

    rclcpp::Subscription<interfaces::msg::JoystickOrder>::SharedPtr subscriber_autonomous_mode_;
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_start_detection;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_detect_parking;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_parking_ok_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_leaving_ok_;
    
    //Attributes
    size_t count_;
    int mode = 0;
    bool search_in_progress = false;
    bool parking_in_progress = false;
    bool parked = false;
    bool leaving_in_progress = false;

    bool manual = true;
    
    interfaces::msg::JoystickOrder car_order;

	  float distance_to_add = 0.0; 
    float distance_travelled = 0.0;

    int parking_type = -1;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutonomousDriving>());
  rclcpp::shutdown();
  return 0;
}