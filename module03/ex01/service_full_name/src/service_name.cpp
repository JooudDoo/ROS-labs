#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "service_full_name/srv/name.hpp"

#include <memory>

void concat_full_name(const std::shared_ptr<service_full_name::srv::Name::Request> request, std::shared_ptr<service_full_name::srv::Name::Response> response){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Request data: %s %s %s", request->first_name.c_str(), request->name.c_str(), request->last_name.c_str());
    response->full_name = request->first_name + " " + request->name + " " + request->last_name;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending data: %s", response->full_name.c_str());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("service_name");

  rclcpp::Service<service_full_name::srv::Name>::SharedPtr service = node->create_service<service_full_name::srv::Name>("full_name", &concat_full_name);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to make names.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}