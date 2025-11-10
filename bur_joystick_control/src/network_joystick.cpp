#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <sstream>
#include <vector>
#include <string>

class NetworkJoystick : public rclcpp::Node
{
public:
    NetworkJoystick() : Node("network_joystick")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);
        
        // Create UDP socket
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return;
        }
        
        // Set up server address
        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(8888);
        
        // Bind socket
        if (bind(sockfd_, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
            close(sockfd_);
            return;
        }
        
        // Start receiving thread
        receiver_thread_ = std::thread(&NetworkJoystick::receive_loop, this);
        
        RCLCPP_INFO(this->get_logger(), "Network joystick receiver started on port 8888");
    }
    
    ~NetworkJoystick() {
        running_ = false;
        if (receiver_thread_.joinable()) {
            receiver_thread_.join();
        }
        if (sockfd_ >= 0) {
            close(sockfd_);
        }
    }

private:
    void receive_loop() {
        char buffer[1024];
        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        
        while (rclcpp::ok() && running_) {
            int bytes_received = recvfrom(sockfd_, buffer, sizeof(buffer)-1, 0,
                                         (sockaddr*)&client_addr, &client_len);
            
            if (bytes_received > 0) {
                buffer[bytes_received] = '\0';
                RCLCPP_DEBUG(this->get_logger(), "Raw JSON: %s", buffer);
                process_data(buffer);
            } else {
                // Small sleep to prevent CPU spinning
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }
    
    void process_data(const char* json_str) {
        auto joy_msg = sensor_msgs::msg::Joy();
        std::string data_str(json_str);
        
        RCLCPP_DEBUG(this->get_logger(), "Processing: %s", data_str.c_str());
        
        try {
            // Parse axes array - look for "axes":[ ... ]
            size_t axes_start = data_str.find("\"axes\":");
            if (axes_start != std::string::npos) {
                axes_start = data_str.find("[", axes_start);
                if (axes_start != std::string::npos) {
                    size_t axes_end = data_str.find("]", axes_start);
                    if (axes_end != std::string::npos) {
                        std::string axes_str = data_str.substr(axes_start + 1, axes_end - axes_start - 1);
                        RCLCPP_DEBUG(this->get_logger(), "Axes string: '%s'", axes_str.c_str());
                        parse_float_array(axes_str, joy_msg.axes);
                    }
                }
            }
            
            // Parse buttons array - look for "buttons":[ ... ]
            size_t buttons_start = data_str.find("\"buttons\":");
            if (buttons_start != std::string::npos) {
                buttons_start = data_str.find("[", buttons_start);
                if (buttons_start != std::string::npos) {
                    size_t buttons_end = data_str.find("]", buttons_start);
                    if (buttons_end != std::string::npos) {
                        std::string buttons_str = data_str.substr(buttons_start + 1, buttons_end - buttons_start - 1);
                        RCLCPP_DEBUG(this->get_logger(), "Buttons string: '%s'", buttons_str.c_str());
                        parse_int_array(buttons_str, joy_msg.buttons);
                    }
                }
            }
            
            // Only publish if we have data
            if (!joy_msg.axes.empty() || !joy_msg.buttons.empty()) {
                // Add header with timestamp
                joy_msg.header.stamp = this->now();
                joy_msg.header.frame_id = "joystick";
                
                publisher_->publish(joy_msg);
                
                RCLCPP_INFO(this->get_logger(), "Published: %zu axes, %zu buttons", 
                           joy_msg.axes.size(), joy_msg.buttons.size());
            }
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error parsing data: %s", e.what());
        }
    }
    
    void parse_float_array(const std::string& str, std::vector<float>& result) {
        std::stringstream ss(str);
        std::string item;
        
        while (std::getline(ss, item, ',')) {
            // Remove any whitespace or brackets
            item.erase(remove_if(item.begin(), item.end(), ::isspace), item.end());
            if (!item.empty()) {
                try {
                    result.push_back(std::stof(item));
                } catch (const std::exception& e) {
                    RCLCPP_DEBUG(this->get_logger(), "Failed to parse float: '%s'", item.c_str());
                }
            }
        }
    }
    
    void parse_int_array(const std::string& str, std::vector<int>& result) {
        std::stringstream ss(str);
        std::string item;
        
        while (std::getline(ss, item, ',')) {
            // Remove any whitespace or brackets
            item.erase(remove_if(item.begin(), item.end(), ::isspace), item.end());
            if (!item.empty()) {
                try {
                    result.push_back(std::stoi(item));
                } catch (const std::exception& e) {
                    RCLCPP_DEBUG(this->get_logger(), "Failed to parse int: '%s'", item.c_str());
                }
            }
        }
    }
    
    int sockfd_ = -1;
    std::thread receiver_thread_;
    bool running_ = true;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NetworkJoystick>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}