#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class Text_to_cmd_vel : public rclcpp::Node {
public:
    Text_to_cmd_vel() : Node("text_to_cmd_vel") {
        cmd_text_subscription_ = this->create_subscription<std_msgs::msg::String>("cmd_text", 10, std::bind(&Text_to_cmd_vel::callback, this, _1));
        to_turtle_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Succesfully create Text_to_cmd_vel node");
    }
private:
    void callback(const std_msgs::msg::String & msg){
        RCLCPP_INFO(this->get_logger(), "Get message from cmd_text: %s", msg.data.c_str());
        std::string cmd_string = msg.data;
        geometry_msgs::msg::Twist turtle_msg;

        if(cmd_string=="turn_right"){
            turtle_msg.angular.z = -1.5;
        }
        else if(cmd_string=="turn_left"){
            turtle_msg.angular.z = 1.5;
        }
        else if(cmd_string=="move_forward"){
            turtle_msg.linear.x = 1.0;
        }
        else if(cmd_string=="move_backward"){
            turtle_msg.linear.x = -1.0;
        }
        RCLCPP_INFO(this->get_logger(), "Send message to turtle1/cmd_vel: %f %f", turtle_msg.angular.z, turtle_msg.linear.x);
        to_turtle_publisher_->publish(turtle_msg);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_text_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr to_turtle_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Text_to_cmd_vel>());
  rclcpp::shutdown();
  return 0;
}