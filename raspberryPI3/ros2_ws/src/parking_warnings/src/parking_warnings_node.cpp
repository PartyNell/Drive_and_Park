#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class ParkingWarningNode : public rclcpp::Node {
public:
    ParkingWarningNode()
    : Node("parking_warning_node"), light_state_(false), is_flashing_(true) {
        // Déclarez un publisher pour publier l'état de la lumière
        light_publisher_ = this->create_publisher<std_msgs::msg::Bool>("parking_warning_light", 10);

        // Créez un timer pour gérer le clignotement périodique
        auto timer_callback = [this]() -> void {
            if (is_flashing_) {
                toggle_light();
            }
        };

        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), timer_callback); // 500 ms intervalle
        RCLCPP_INFO(this->get_logger(), "Parking Warning Node has started.");
    }

private:
    void toggle_light() {

        light_state_ = !light_state_;

        auto message = std_msgs::msg::Bool();
        message.data = light_state_;
        light_publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Light is now %s", light_state_ ? "ON" : "OFF");
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr light_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool light_state_; 
    bool is_flashing_; 
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParkingWarningNode>());
    rclcpp::shutdown();
    return 0;
}
