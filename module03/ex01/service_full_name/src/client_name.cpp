#include "rclcpp/rclcpp.hpp"
#include "service_full_name/srv/name.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 4) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: first_name name last_name");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("name_client");
  rclcpp::Client<service_full_name::srv::Name>::SharedPtr client = node->create_client<service_full_name::srv::Name>("full_name");

  auto request = std::make_shared<service_full_name::srv::Name::Request>();
  request->first_name = (argv[1]);
  request->name = (argv[2]);
  request->last_name = (argv[3]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Full name: %s", result.get()->full_name.c_str());
  } 
  else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service service_full_name");
  }

  rclcpp::shutdown();
  return 0;
}