#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "bur_msgs/msg/thruster_command.hpp"

using namespace std::placeholders;

class JoystickConverter : public rclcpp::Node
{
public:
    JoystickConverter() : Node("joystick_converter")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoystickConverter::topic_callback, this, _1));
        
        thruster_pub_ = this->create_publisher<bur_msgs::msg::ThrusterCommand>("thruster_command", 10);
        
        RCLCPP_INFO(this->get_logger(), "Joystick converter started");
    }

private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Create thruster command message
        auto thrust_msg = bur_msgs::msg::ThrusterCommand();
        
        // PS4 Controller Mapping:
        // axes[0] = Left Stick X (-1 left, 1 right)
        // axes[1] = Left Stick Y (-1 up, 1 down) - NOTE: Inverted!
        // axes[2] = Right Stick X
        // axes[3] = Right Stick Y
        // axes[4] = L2 Trigger (0 to 1)
        // axes[5] = R2 Trigger (0 to 1)
        
        // Buttons:
        // buttons[0] = X (A)
        // buttons[1] = Circle (B)
        // buttons[2] = Square (X)
        // buttons[3] = Triangle (Y)
        // buttons[4] = L1
        // buttons[5] = R1
        // buttons[6] = L2
        // buttons[7] = R2
        // buttons[8] = Share
        // buttons[9] = Options
        // buttons[10] = PS Button
        // buttons[11] = Left Stick Press
        // buttons[12] = Right Stick Press
        
        // Extract control values
        float left_stick_x = msg->axes[0];    // Turning
        float left_stick_y = -msg->axes[1];   // Forward/backward (invert Y axis)
        float right_stick_x = msg->axes[2];   // Strafe left/right
        float right_stick_y = -msg->axes[3];  // Up/down (invert Y axis)
        float l2_trigger = msg->axes[4];      // Alternative up/down
        float r2_trigger = msg->axes[5];      // Alternative up/down
        
        // Choose vertical control - prefer right stick, fall back to triggers
        float vertical = 0.0;
        if (fabs(right_stick_y) > 0.1) {
            vertical = right_stick_y;  // Right stick for precise vertical
        } else {
            // Use triggers for vertical: R2 up, L2 down
            vertical = r2_trigger - l2_trigger;
        }
        
        // Dead zones - ignore small movements
        if (fabs(left_stick_x) < 0.1) left_stick_x = 0.0;
        if (fabs(left_stick_y) < 0.1) left_stick_y = 0.0;
        if (fabs(right_stick_x) < 0.1) right_stick_x = 0.0;
        if (fabs(vertical) < 0.1) vertical = 0.0;
        
        // Scale for better control
        left_stick_x *= 1.5f;   // More responsive turning
        left_stick_y *= 1.0f;   // Normal forward/backward
        right_stick_x *= 1.0f;  // Normal strafing
        vertical *= 1.0f;       // Normal vertical
        
        // 8-thruster mapping for underwater vehicle:
        // Thrusters 1-4: Horizontal movement (forward/backward, turning, strafing)
        // Thrusters 5-8: Vertical movement
        
        thrust_msg.thrusters = {
            // Front-left quadrant
            left_stick_y + left_stick_x + right_stick_x,  // Thruster 1
            // Front-right quadrant  
            left_stick_y - left_stick_x - right_stick_x,  // Thruster 2
            // Back-left quadrant
            -left_stick_y + left_stick_x - right_stick_x, // Thruster 3
            // Back-right quadrant
            -left_stick_y - left_stick_x + right_stick_x, // Thruster 4
            // Vertical thrusters
            vertical,  // Thruster 5
            vertical,  // Thruster 6
            vertical,  // Thruster 7
            vertical   // Thruster 8
        };
        
        // Limit thruster values to [-1.0, 1.0]
        for (auto& thrust : thrust_msg.thrusters) {
            thrust = std::max(-1.0f, std::min(1.0f, thrust));
        }
        
        thruster_pub_->publish(thrust_msg);
        
        // Debug output for significant movements
        if (fabs(left_stick_y) > 0.2 || fabs(left_stick_x) > 0.2 || 
            fabs(vertical) > 0.2 || fabs(right_stick_x) > 0.2) {
            RCLCPP_DEBUG(this->get_logger(), 
                        "Controls: F/B:%.2f, Turn:%.2f, Vert:%.2f, Strafe:%.2f", 
                        left_stick_y, left_stick_x, vertical, right_stick_x);
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<bur_msgs::msg::ThrusterCommand>::SharedPtr thruster_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickConverter>());
    rclcpp::shutdown();
    return 0;
}

// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>

// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/joy.hpp"
// #include "bur_msgs/msg/thruster_command.hpp"

// class JoystickConverter : public rclcpp::Node
// {
// public:
//     JoystickConverter() : Node("joystick_converter")
//     {
//         joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
//             "joy", 10, std::bind(&JoystickConverter::joy_callback, this, std::placeholders::_1));
        
//         // reads from joystick, publishes to thruster command
//         thruster_pub_ = this->create_publisher<bur_msgs::msg::ThrusterCommand>("thruster_command", 10);
        
//         RCLCPP_INFO(this->get_logger(), "Joystick converter started");
//     }

// private:
//     void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
//     {
//         auto thrust_msg = bur_msgs::msg::ThrusterCommand();
        
//         // Map joystick axes to thrusters
//         //TODO: adjust mapping and how to control
        
//         /*
//         I don't really have access to a controller, so this is what ChatGPT says:
//         axes[0] - Left stick X (left/right)
//         axes[1] - Left stick Y (up/down)
//         axes[2] - Right stick X
//         axes[3] - Right stick Y
//         axes[4] - Right trigger
//         axes[5] - Left trigger
//         */
        
//         float forward = msg->axes[1];
//         float turn = msg->axes[0];
//         float vertical = msg->axes[4];
        
//         // I'm not quite sure how to map this, so this is a guess from ChatGPT :)
//         thrust_msg.thrusters = {
//             forward + turn,    // Thruster 1
//             forward - turn,    // Thruster 2  
//             -forward + turn,   // Thruster 3
//             -forward - turn,   // Thruster 4
//             vertical,          // Thruster 5
//             vertical,          // Thruster 6
//             vertical,          // Thruster 7
//             vertical           // Thruster 8
//         };
        
//         thruster_pub_->publish(thrust_msg);
//     }
    
//     rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
//     rclcpp::Publisher<bur_msgs::msg::ThrusterCommand>::SharedPtr thruster_pub_;
// };

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<JoystickConverter>());
//     rclcpp::shutdown();
//     return 0;
// }