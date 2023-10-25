
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


class HeartMove : public rclcpp::Node{

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    constexpr static const float x_step = 0.01;

    float x_val;
    int part;

public:
    HeartMove(const rclcpp::NodeOptions& options) : Node("circle_move", options){
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Succesfully create HeartMove node");

        part = 0;
        x_val = -1;
        timer_ = create_wall_timer(std::chrono::milliseconds(150), std::bind(&HeartMove::timer_callback, this));
    }

private:

    void timer_callback(){
        geometry_msgs::msg::Twist msg = geometry_msgs::msg::Twist();
        x_val += x_step;
        if(x_val > 1) {
            x_val = -1;
            part = (part + 1) % 2;
        }
        float y_val =0;
        if(part == 0){
            y_val = powf(x_val, 2/3) + sqrtf(1-pow(x_val, 2));
        }
        else{
             y_val = powf(x_val, 2/3) - sqrtf(1-pow(x_val, 2));
        }


        msg.angular.z = y_val;
        msg.linear.x = x_val;
        RCLCPP_INFO(this->get_logger(), "Next moving: %f %f", msg.angular.z, msg.linear.x);
        cmd_vel_pub_->publish(msg);
    }

};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<HeartMove>(rclcpp::NodeOptions()));

    rclcpp::shutdown();
    return 0;

}