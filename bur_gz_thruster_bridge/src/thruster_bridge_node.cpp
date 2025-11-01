#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "bur_msgs/msg/thruster_command.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ThrusterBridge : public rclcpp::Node
{
  public:
    ThrusterBridge()
    : Node("thruster_bridge")
    {
      subscription_ = this->create_subscription<bur_msgs::msg::ThrusterCommand>(
        "thruster_command", 10, 
        std::bind(&ThrusterBridge::thruster_callback, this, std::placeholders::_1));

        for (int i = 0; i < 8; i++) {
            std::string topic_name = "model/bur/joint/thruster_" + std::to_string(i+1) + "/cmd_pos";
            auto publisher = this->create_publisher<std_msgs::msg::Float64>(topic_name, 10);
            publishers_.push_back(publisher);
        }

        RCLCPP_INFO(this->get_logger(), "Thruster bridge started");
        RCLCPP_INFO(this->get_logger(), "Subscribed to 'thruster_command'");
        RCLCPP_INFO(this->get_logger(), "Publishing to 8 Gazebo thruster topics");
    }

  private:
    void thruster_callback(const bur_msgs::msg::ThrusterCommand::SharedPtr msg)
    {
      if (msg->thrusters.size() != publishers_.size()) {
        RCLCPP_WARN(this->get_logger(), 
                   "Received %zu thrusters but expected %zu", 
                   msg->thrusters.size(), publishers_.size());
        return;
      }
      
      for (size_t i = 0; i < msg->thrusters.size(); i++) {
        auto thrust_msg = std_msgs::msg::Float64();
        thrust_msg.data = msg->thrusters[i];
        publishers_[i]->publish(thrust_msg);
      }
      
      RCLCPP_INFO(this->get_logger(), "Published thruster commands to Gazebo");
    }
    
    rclcpp::Subscription<bur_msgs::msg::ThrusterCommand>::SharedPtr subscription_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> publishers_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThrusterBridge>());
  rclcpp::shutdown();
  return 0;
}