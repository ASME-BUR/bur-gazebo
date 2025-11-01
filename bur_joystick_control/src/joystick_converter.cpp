#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "bur_msgs/msg/thruster_command.hpp"

class JoystickConverter : public rclcpp::Node
{
public:
    JoystickConverter() : Node("joystick_converter")
    {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoystickConverter::joy_callback, this, std::placeholders::_1));
        
        // reads from joystick, publishes to thruster command
        thruster_pub_ = this->create_publisher<bur_msgs::msg::ThrusterCommand>("thruster_command", 10);
        
        RCLCPP_INFO(this->get_logger(), "Joystick converter started");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto thrust_msg = bur_msgs::msg::ThrusterCommand();
        
        // Map joystick axes to thrusters
        //TODO: adjust mapping and how to control
        
        /*
        I don't really have access to a controller, so this is what ChatGPT says:
        axes[0] - Left stick X (left/right)
        axes[1] - Left stick Y (up/down)
        axes[2] - Right stick X
        axes[3] - Right stick Y
        axes[4] - Right trigger
        axes[5] - Left trigger
        */
        
        float forward = msg->axes[1];
        float turn = msg->axes[0];
        float vertical = msg->axes[4];
        
        // I'm not quite sure how to map this, so this is a guess from ChatGPT :)
        thrust_msg.thrusters = {
            forward + turn,    // Thruster 1
            forward - turn,    // Thruster 2  
            -forward + turn,   // Thruster 3
            -forward - turn,   // Thruster 4
            vertical,          // Thruster 5
            vertical,          // Thruster 6
            vertical,          // Thruster 7
            vertical           // Thruster 8
        };
        
        thruster_pub_->publish(thrust_msg);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<bur_msgs::msg::ThrusterCommand>::SharedPtr thruster_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickConverter>());
    rclcpp::shutdown();
    return 0;
}