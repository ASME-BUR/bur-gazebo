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

        /*
        Axis 0 : Left Stick X (-1 Left, 1 Right)
        Axis 1 : Left Stick Y (-1 Up, 1 Down)
        Axis 2 : Right Stick X (-1 Left, 1 Right)
        Axis 3 : Right Stick Y (-1 Up, 1 Down)
        Axis 4 : L2 Trigger (-1 Not Pressed, 1 Fully Pressed)
        Axis 5 : R2 Trigger (-1 Not Pressed, 1 Fully Pressed)
        Button 0 : X
        Button 1 : Circle
        Button 2 : Square
        Button 3 : Triangle
        Button 4 : Share
        Button 5 : PS Button
        Button 6 : Options
        Button 7 : L3 (Left Stick Press)
        Button 8 : R3 (Right Stick Press)
        Button 9 : L1 Bumper
        Button 10 : R1 Bumper
        Button 11 : D-Pad Up
        Button 12 : D-Pad Down
        Button 13 : D-Pad Left
        Button 14 : D-Pad Right
        Button 15 : Touchpad
        */

        /*
        PS4 Controller Mapping:
        Axis 0 : Left Stick X (-1 Left, 1 Right)        -> Strafe Left/Right
        Axis 1 : Left Stick Y (-1 Up, 1 Down)           -> Forward/Backward (inverted)
        Axis 2 : Right Stick X (-1 Left, 1 Right)       -> Rotate Left/Right (Yaw)
        Axis 3 : Right Stick Y (-1 Up, 1 Down)          -> Rotate Up/Down (Pitch)
        Axis 4 : L2 Trigger (-1 Not Pressed, 1 Fully Pressed) -> Go Down
        Axis 5 : R2 Trigger (-1 Not Pressed, 1 Fully Pressed) -> Go Up
        */
        
        // Extract control values with dead zones
        float strafe = apply_deadzone(msg->axes[0], 0.1f);        // Left Stick X: Strafe left/right
        float forward = apply_deadzone(-msg->axes[1], 0.1f);      // Left Stick Y: Forward/backward (inverted)
        float yaw = apply_deadzone(msg->axes[2], 0.1f);           // Right Stick X: Rotate left/right
        float pitch = apply_deadzone(-msg->axes[3], 0.1f);        // Right Stick Y: Rotate up/down (inverted)
        float down = apply_deadzone((msg->axes[4] + 1.0f) / 2.0f, 0.1f);  // L2: 0 to 1 range for down
        float up = apply_deadzone((msg->axes[5] + 1.0f) / 2.0f, 0.1f);    // R2: 0 to 1 range for up
        
        // Calculate vertical movement (up/down)
        float vertical = up - down;  // R2 makes it go up, L2 makes it go down
        
        // Apply scaling for better control feel
        strafe *= 1.0f;
        forward *= 1.0f;
        yaw *= 1.5f;       // More responsive rotation
        pitch *= 1.0f;
        vertical *= 1.0f;
        
        // 8-thruster mapping for underwater vehicle:
        // Assuming thrusters are arranged like:
        //   1     2
        //     ↑ ↑
        //   3     4
        //
        //   5     6
        //   7     8
        // Thrusters 1-4: Horizontal movement (forward/backward, strafing, yaw)
        // Thrusters 5-8: Vertical movement (vertical movement, pitch)
        
        thrust_msg.thrusters = {
            // Front-left thruster (1)
            forward + strafe + yaw,
            // Front-right thruster (2)
            forward - strafe - yaw
            // Back-left thruster (3)
            -forward + strafe - yaw
            // Back-right thruster (4)
            -forward - strafe + yaw,
            // Front vertical thruster (5)
            vertical + pitch,
            // Front vertical thruster (6)
            vertical + pitch,
            // Back vertical thruster (7)
            vertical - pitch,
            // Back vertical thruster (8)
            vertical - pitch
        };
        
        // Limit thruster values to [-1.0, 1.0]
        for (auto& thrust : thrust_msg.thrusters) {
            thrust = std::max(-1.0f, std::min(1.0f, thrust));
        }
        
        thruster_pub_->publish(thrust_msg);
        
        // Debug output for significant movements
        // if (fabs(left_stick_y) > 0.2 || fabs(left_stick_x) > 0.2 || 
        //     fabs(vertical) > 0.2 || fabs(right_stick_x) > 0.2) {
        //     RCLCPP_DEBUG(this->get_logger(), 
        //                 "Controls: F/B:%.2f, Turn:%.2f, Vert:%.2f, Strafe:%.2f", 
        //                 left_stick_y, left_stick_x, vertical, right_stick_x);
        // }
    }

    float apply_deadzone(float value, float deadzone) {
        if (fabs(value) < deadzone) {
            return 0.0f;
        }
        return value;
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