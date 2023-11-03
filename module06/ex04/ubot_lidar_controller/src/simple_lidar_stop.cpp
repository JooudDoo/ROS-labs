#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class SimpleLidarController : public rclcpp::Node {

    using TwistMsg = geometry_msgs::msg::Twist;
    using LScanMsg = sensor_msgs::msg::LaserScan;

    rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<LScanMsg>::SharedPtr laser_scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool isInited = false;

public:
    SimpleLidarController(const rclcpp::NodeOptions& options) : Node("simple_lidar_node", options) {
        cmd_vel_pub_ = this->create_publisher<TwistMsg>("/robot/cmd_vel", 10);
        laser_scan_sub_ = this->create_subscription<LScanMsg>("/robot/scan", 10, std::bind(&SimpleLidarController::callbackLScan, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Succesfully created node");

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SimpleLidarController::checkLidar, this));
    }


private:

    void callbackLScan(const LScanMsg& msg){  }

    void getLScanMsg(LScanMsg* msg){
        rclcpp::MessageInfo minfo;
        while(!laser_scan_sub_->take(*msg, minfo));
    }

    void checkLidar(){
        LScanMsg lidarData;
        getLScanMsg(&lidarData);

        TwistMsg velMsg; 

        velMsg.linear.x = 1;

        for(size_t msgId = 0; msgId < lidarData.ranges.size(); msgId += 1){
            if(lidarData.ranges[msgId] < 1){
                velMsg.linear.x = 0;
                isInited = true;
                break;
            }
        }

        if(isInited){
            cmd_vel_pub_->publish(velMsg);
        }

    }

};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleLidarController>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}