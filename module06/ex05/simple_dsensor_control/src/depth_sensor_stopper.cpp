#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"

class SimpleDSensorController : public rclcpp::Node {

    using TwistMsg = geometry_msgs::msg::Twist;
    using DSensorMsg = sensor_msgs::msg::Image;

    rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<DSensorMsg>::SharedPtr dsensor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::deque<std::vector<unsigned char>> previousData;
    std::deque<unsigned char> previousVals;

    bool isInited = false;

public:
    SimpleDSensorController(const rclcpp::NodeOptions& options) : Node("simple_dsensor_node", options) {
        cmd_vel_pub_ = this->create_publisher<TwistMsg>("/robot/cmd_vel", 10);
        dsensor_sub_ = this->create_subscription<DSensorMsg>("/depth/image", 10, std::bind(&SimpleDSensorController::callbackDsensor, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Succesfully created node");

        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&SimpleDSensorController::checkDsensor, this));
    }

private:
    void callbackDsensor(const DSensorMsg& msg){  }

    void getDsensorMsg(DSensorMsg* msg){
        rclcpp::MessageInfo minfo;
        while(!dsensor_sub_->take(*msg, minfo));
    }
    
    unsigned char meanpreviousVals(){
        unsigned char mean;
        for(auto i : previousVals){
            mean += i;
        }
        return mean / previousVals.size();
    }

    void checkDsensor(){
        DSensorMsg dsensorImage;
        getDsensorMsg(&dsensorImage);

        TwistMsg velMsg; 

        velMsg.linear.x = 0.5;

        if(previousData.size() < 3){
            previousData.push_back(dsensorImage.data);
            return;   
        }

        previousData.pop_front();
        previousData.push_back(dsensorImage.data);


        if(dsensorImage.width){
            
            unsigned char targetPixelMeanVal;

            for(size_t im_id = 0; im_id < previousData.size(); im_id++){
                targetPixelMeanVal += previousData[im_id][(dsensorImage.width*dsensorImage.height/2+dsensorImage.width/2)];
            }
            targetPixelMeanVal /= previousData.size();
            previousVals.push_back(targetPixelMeanVal);

            if(previousVals.size()>3){
                previousVals.pop_front();
            }

            RCLCPP_INFO(this->get_logger(), "Val: %u", meanpreviousVals());

            if(meanpreviousVals() != 0){
                velMsg.linear.x = 0;
                isInited = true;
            }

        }
        
        if(isInited){
            cmd_vel_pub_->publish(velMsg);
        }

    }

};


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleDSensorController>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}