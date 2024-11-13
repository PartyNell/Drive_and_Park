#include <stdint.h>
#include <string.h>  

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

#include "interfaces/msg/joystick_order.hpp"
#include "interfaces/msg/system_check.hpp"


using namespace std;
using placeholders::_1;

class car_command : public rclcpp::Node {

public:
    car_command()
    : Node("jacr_command_node")
    {
        
        publisher_car_order_= this->create_publisher<interfaces::msg::JoystickOrder>("car_order", 10);
        publisher_system_check_= this->create_publisher<interfaces::msg::SystemCheck>("system_check", 10);


        
        subscription_joystick_order = this->create_subscription<interfaces::msg::JoystickOrder>(
        "joystick_order", 10, std::bind(&..., this, _1)); 
        //complete bind with the function we want to execute when a value has been receved

        
        RCLCPP_INFO(this->get_logger(), "joystick_to_cmd_node READY");
    }

    
private:

    //Update requestedThrottle, requestedAngle and reverse from the joystick
    void joyCallback(const sensor_msgs::msg::Joy & joy) {

        buttonStart = joy.buttons[buttonsMap.find("START")->second];
        buttonB = joy.buttons[buttonsMap.find("B")->second];    
        buttonA = joy.buttons[buttonsMap.find("A")->second];  
        buttonY = joy.buttons[buttonsMap.find("Y")->second]; 
        

        axisRT = joy.axes[axisMap.find("RT")->second];      //Motors (go forward)
        axisLT = joy.axes[axisMap.find("LT")->second];      //Motors (go backward)
        axisLS_X = joy.axes[axisMap.find("LS_X")->second];     //Steering

        if (joy.axes[axisMap.find("DPAD_Y")->second] == -1.0)
            buttonDpadBottom = true;
        else
            buttonDpadBottom = false;

        if (joy.axes[axisMap.find("DPAD_X")->second] == 1.0)
            buttonDpadLeft = true;
        else
            buttonDpadLeft = false;
        
        


        //Normalise values
        axisLS_X = -axisLS_X;   //axisLS_X : 1 .. -1  ;  steering_angle : -1 .. 1
        axisRT = (1.0-axisRT)/2.0;  //axisRT : 1 .. -1  ;  throttle : 0 .. 1
        axisLT = (1.0-axisLT)/2.0;  //axisLT : 1 .. -1  ;  throttle : 0 .. 1


        //Select mode (0 : manual ; 1 : autonomous ; 2 : steering calibration)
        if (mode == 2) //Exit steering calibration mode after request has been sent
            mode = -1;
        

        if (buttonA || buttonY || buttonDpadBottom){

            if (buttonY)
                mode = 0;
            else if (buttonA)
                mode = 1;
            else if (buttonDpadBottom && buttonStart){
                mode = 2;
                start = false;
            }
        }

        if (buttonDpadLeft && !systemCheckPrintRequest){ //Request to print the last system check report
            systemCheckPrintRequest = true;

            auto systemCheckMsg = interfaces::msg::SystemCheck();
            systemCheckMsg.print = true;
            publisher_system_check_->publish(systemCheckMsg); //Send print request to system_check_node
        }
        else
            systemCheckPrintRequest = false;


        // ------ Start and Stop ------
        if (buttonB){       // B button -> Stop the car
            start = false;

        }else if (buttonStart && mode !=2){   // Start button -> Start the car    
            start = true;
        }


        // ------ Propulsion ------
        if (axisLT > DEADZONE_LT_RT && axisRT > DEADZONE_LT_RT){  //Incompatible orders : Stop the car
            requestedThrottle = STOP;
            RCLCPP_WARN(this->get_logger(), "Incompatible orders : LT = %f, RT = %f",axisLT,axisRT);

        } else if (axisLT < DEADZONE_LT_RT && axisRT < DEADZONE_LT_RT){ 
            requestedThrottle = STOP;
        
        }else if (axisLT > DEADZONE_LT_RT){ //Move backward
            reverse = true;
            requestedThrottle = axisLT;

        } else if (axisRT > DEADZONE_LT_RT){   //Move forward
            reverse = false;
            requestedThrottle = axisRT;
        }


        // ------ Steering ------
        if (axisLS_X > DEADZONE_LS_X_LEFT && axisLS_X < DEADZONE_LS_X_RIGHT){     //asymmetric deadzone (hardware : joystick LS)
            requestedAngle = CENTER;
        } else {
            requestedAngle = axisLS_X;    
        }

        

        auto joystickOrderMsg = interfaces::msg::JoystickOrder();
        joystickOrderMsg.start = start;
        joystickOrderMsg.mode = mode;
        joystickOrderMsg.throttle = requestedThrottle;
        joystickOrderMsg.steer  = requestedAngle;
        joystickOrderMsg.reverse = reverse;

        publisher_joystick_order_->publish(joystickOrderMsg); //Send order to the car_control_node
    }

    //Joystick variables
    map<string,int> axisMap;
    map<string,int> buttonsMap;
    bool buttonB, buttonStart, buttonA, buttonY, buttonDpadBottom, buttonDpadLeft ;
    
    float axisRT, axisLT, axisLS_X;

    //General variables
    bool start;
    int mode;
    bool systemCheckPrintRequest;


    //Manual mode variables
    float requestedAngle, requestedThrottle;
    bool reverse;



    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr publisher_joystick_order_;
    rclcpp::Publisher<interfaces::msg::SystemCheck>::SharedPtr publisher_system_check_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_joy_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<joystick_to_cmd>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}