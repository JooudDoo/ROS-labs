
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


class CircleMove : public rclcpp::Node{

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    CircleMove(const rclcpp::NodeOptions& options) : Node("circle_move", options){
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Succesfully create CircleMove node");

        timer_ = create_wall_timer(std::chrono::milliseconds(150), std::bind(&CircleMove::timer_callback, this));
    }

private:

    void timer_callback(){
        geometry_msgs::msg::Twist msg = geometry_msgs::msg::Twist();
        msg.angular.z = -2.0;
        msg.linear.x = 5.0;
        RCLCPP_INFO(this->get_logger(), "Next moving: %f %f", msg.angular.z, msg.linear.x);
        cmd_vel_pub_->publish(msg);
    }

};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<CircleMove>(rclcpp::NodeOptions()));

    rclcpp::shutdown();
    return 0;

}