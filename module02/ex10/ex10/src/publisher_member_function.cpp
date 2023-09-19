#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    while(process_message());
  }

private:
  bool process_message(){
    geometry_msgs::msg::Twist msg;
    std::string console_input;
    std::getline(std::cin, console_input);
    if(console_input=="turn_right"){
      msg.angular.z = -1.5;
    }
    else if(console_input=="turn_left"){
      msg.angular.z = 1.5;
    }
    else if(console_input=="move_forward"){
      msg.linear.x = 1.0;
    }
    else if(console_input=="move_backward"){
      msg.linear.x = -1.0;
    }
    else if(console_input=="exit"){
      return false;
    }
    publisher_->publish(msg);
    return true;
  }

  // void timer_callback()
  // {
  //   auto message = std_msgs::msg::String();
  //   message.data = "Hello, world! " + std::to_string(count_++);
  //   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  //   publisher_->publish(message);
  // }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
