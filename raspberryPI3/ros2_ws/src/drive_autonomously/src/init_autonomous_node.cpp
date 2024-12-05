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
        clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
        previous_time = clock_->now().seconds();

        RCLCPP_INFO(this->get_logger(), "init_autonomous_node READY");
    }

private:
    void timer_callback()
    {
        if(init_state){
            float steer = (2*R_MIN*angle_to_perform)/(throttle_order*CAR_SPEED*T0);
            if(steer < -1.0){
                steer_order = -1.0;
            } else if(steer > 1.0){
                steer_order = 1.0;
            } else {
                steer_order = steer;
            }

            RCLCPP_INFO(this->get_logger(), "STEER ORDER : %f", steer_order);

            auto car_order = interfaces::msg::JoystickOrder();
            car_order.start = true;
            car_order.mode = 1;
            car_order.throttle = throttle_order;
            car_order.steer = steer_order;
            car_order.reverse = false;
            
            publisher_car_order_->publish(car_order);

            if(abs(steer_order) < 0.05){
                ++validation_counter;
                if(validation_counter == 5) {
                    std_msgs::msg::Bool finished;
                    finished.data = true;
                    publisher_init_finished_->publish(finished);

                    validation_counter = 0;
                    init_state = false;
                }
            } else {
                validation_counter = 0;
            }
        }
    }

    void start_init_order(const std_msgs::msg::Bool & i){
        RCLCPP_INFO(this->get_logger(), "START Initialisation");
        init_state = i.data;
    }  

    void compute_angle(const interfaces::msg::Ultrasonic & us) {
        //compute the angle between the initial position of the car and the aligned position
        RCLCPP_INFO(this->get_logger(), "ULTRASONIC : front= %d, back=%d", us.front_right, us.rear_right);

        if(us.front_right <= 200 && us.rear_right <= 200){
            tmp_angle_to_perform = atan(static_cast<float>(us.front_right-us.rear_right)/CAR_SIZE);

            //compute the maximum angle that could be performed : robustness for the holes between the cars
            actual_time = rclcpp::Clock().now().seconds();
            delta_time = actual_time - previous_time;

            maximal_angle_perform = (steer_order*throttle_order*CAR_SPEED)/(2*R_MIN);

            //save the angle to perform saturate by the capacities of the car
            if(abs(angle_to_perform - tmp_angle_to_perform) <= maximal_angle_perform){
                angle_to_perform = tmp_angle_to_perform;
            } 
            else {
                angle_to_perform = maximal_angle_perform * copysign(1.0, tmp_angle_to_perform);
            }

            RCLCPP_INFO(this->get_logger(), "ANGLE TO PERFORM : %f", angle_to_perform);

            previous_time = actual_time;
            
        } else if (init_state && (us.front_right > 200 || us.rear_right > 200)) {
            //the wall or the parked car are too far to compute the angle
            init_state = false;

            std_msgs::msg::Bool finished;
            finished.data = false;
            publisher_init_finished_->publish(finished);
        }
    }

    void compute_angle(const interfaces::msg::Ultrasonic & us) {
        //compute the angle between the initial position of the car and the aligned position
        RCLCPP_INFO(this->get_logger(), "ULTRASONIC : front= %d, back=%d", us.front_right, us.rear_right);

        if(us.front_right <= 200 && us.rear_right <= 200){
            tmp_angle_to_perform = atan(static_cast<float>(us.front_right-us.rear_right)/CAR_SIZE);

            //compute the maximum angle that could be performed : robustness for the holes between the cars
            //actual_time = clock_->now().seconds();
            delta_time = actual_time - previous_time;

            maximal_angle_perform = (steer_order*throttle_order*CAR_SPEED*delta_time)/(2*R_MIN);

            if(abs(angle_to_perform - tmp_angle_to_perform) <= maximal_angle_perform){
                angle_to_perform = tmp_angle_to_perform;
            } 
            else {
                angle_to_perform = maximal_angle_perform * copysign(1.0, tmp_angle_to_perform);
            }

            angle_to_perform = atan(static_cast<float>(us.front_right-us.rear_right)/CAR_SIZE);
            RCLCPP_INFO(this->get_logger(), "ANGLE TO PERFORM : %f", angle_to_perform);

            previous_time = actual_time;
            
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
    float throttle_order = 0.7; // throttle sent to the node `car_control_node`
    bool init_state = false;

    int validation_counter = 0;

    float angle_to_perform;

    rclcpp::Clock::SharedPtr clock_;
    float previous_time;
    float actual_time;


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