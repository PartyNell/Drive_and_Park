#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/joystick_order.hpp"
#include "interfaces/msg/ultrasonic.hpp"

#include "std_msgs/msg/bool.hpp"

#define CAR_SIZE 89.0f 
#define CAR_SPEED 1.6f
#define R_MIN 1.7f 
#define T0 0.1 //in seconds

using namespace std::chrono_literals;
using std::placeholders::_1;

class init_autonomous : public rclcpp::Node
{
public:
    init_autonomous()
    : Node("init_autonomous_node"), count_(0)
    {
        publisher_car_order_ = this->create_publisher<interfaces::msg::JoystickOrder>("autonomous_car_order", 10);
        publisher_init_finished_ = this->create_publisher<std_msgs::msg::Bool>("init_finished", 10);

        subscription_start_init_ = this->create_subscription<std_msgs::msg::Bool>("start_init", 10, std::bind(&init_autonomous::start_init_order, this, _1));
        subscription_ultrasonic_ = this->create_subscription<interfaces::msg::Ultrasonic>("/us_data", 10, std::bind(&init_autonomous::compute_angle, this, _1));

        timer_ = this->create_wall_timer(100ms, std::bind(&init_autonomous::timer_callback, this));
        clock_ = rclcpp::Clock(RCL_SYSTEM_TIME);

        RCLCPP_INFO(this->get_logger(), "init_autonomous_node READY");
    }

private:
    void timer_callback()
    {
        if(init_state){

            steer_order = 64*pow(angle_to_perform, 3);

            if(steer_order > 1.0) steer_order=1.0;
            else if(steer_order < -1.0) steer_order=-1.0;
            else if (abs(angle_to_perform) < 0.3) steer_order=0.0;
            else if(steer_order > 0.0 && steer_order < 0.20) steer_order=0.2;
            else if(steer_order < 0.0 && steer_order > -0.20) steer_order=-0.2;

            //RCLCPP_INFO(this->get_logger(), "STEER ORDER : %f", steer_order);

            auto car_order = interfaces::msg::JoystickOrder();
            car_order.start = true;
            car_order.mode = 1;
            car_order.throttle = throttle_order;
            car_order.steer = steer_order;
            car_order.reverse = false;
            
            publisher_car_order_->publish(car_order);

            current_time = clock_.now();

            init_time = current_time - reference_time;

            if(abs(angle_to_perform) < 0.05){
                ++validation_counter;
                if(validation_counter == 5) {
                    std_msgs::msg::Bool finished;
                    finished.data = true;
                    publisher_init_finished_->publish(finished);

                    validation_counter = 0;
                    init_state = false;
                }
            } else if (init_time.seconds() > 10.0){
                std_msgs::msg::Bool finished;
                finished.data = false;
                publisher_init_finished_->publish(finished);

                validation_counter = 0;
                init_state = false;
            } else {
                validation_counter = 0;
            }
        }
    }

    void start_init_order(const std_msgs::msg::Bool & i){
        init_state = i.data;

        if(init_state){
            RCLCPP_INFO(this->get_logger(), "START Initialisation");
            reference_time = clock_.now();
        }
    }  

    void compute_angle(const interfaces::msg::Ultrasonic & us) {
        //compute the angle between the initial position of the car and the aligned position
        if(us.front_right <= 200 && us.rear_right <= 200){

            angle_to_perform = atan(static_cast<float>(us.front_right-us.rear_right)/CAR_SIZE);
            
        } else if (init_state && (us.front_right > 200 || us.rear_right > 200)) {
            //the wall or the parked car are too far to compute the angle
            init_state = false;

            std_msgs::msg::Bool finished;
            finished.data = false;
            publisher_init_finished_->publish(finished);
        }
    }


    // Private variables

    float steer_order; // angle sent to the node `car_control_node`
    float throttle_order = 0.4; // throttle sent to the node `car_control_node`
    bool init_state = false;

    int validation_counter = 0;

    float angle_to_perform;

    rclcpp::Clock clock_;
    rclcpp::Time reference_time;
    rclcpp::Time current_time;
    rclcpp::Duration init_time  = rclcpp::Duration::from_seconds(0);


    // Publisher
    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr publisher_car_order_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_init_finished_;

    //Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_start_init_;
    rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr subscription_ultrasonic_;

    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<init_autonomous>());
    rclcpp::shutdown();
    return 0;
}
