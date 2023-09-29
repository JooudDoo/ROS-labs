
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

#define DISTANCE_ACCURACY  0.01
#define ANGLE_ACCURACY    0.001

class MoveToGoal : public rclcpp::Node {
public:

    MoveToGoal(const rclcpp::NodeOptions& options, int argc, char* argv[]) : Node("move_to", options){
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        pose_sub_= this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&MoveToGoal::callback_cmd_pose, this, std::placeholders::_1));        
        if(argc != 4){
            RCLCPP_ERROR(this->get_logger(), "Wrong amount of arguments. Shutdown..\nUsage: X Y theta");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "Succesfully create Move_to_goal node");
        bool is_moved = move_to_goal(argv);
    }

private:

    void callback_cmd_pose(const turtlesim::msg::Pose& msg) {(&msg);};


    /*
        Sends a turtlesim Twist message and waits for its execution (Only then returns control)
    */
    void send_message_sync(float linear_speed = 0, float angle_speed = 0){
        geometry_msgs::msg::Twist vel_msg;

        vel_msg.linear.x = linear_speed;
        vel_msg.angular.z = angle_speed;
        cmd_vel_pub_->publish(vel_msg);

        is_moving();
    }

    /*
        Sets turtle position to msg pointer refered as arg
    */
    void get_turtle_pos(turtlesim::msg::Pose* msg){
        rclcpp::MessageInfo minfo;
        while(!pose_sub_->take(*msg, minfo));
    }

    /*
        Returns control to the program further as soon as the turtle finishes moving
    */
    void is_moving(){
        turtlesim::msg::Pose c_pos;
        rclcpp::MessageInfo minfo;
        while(true){
            while(!pose_sub_->take(c_pos, minfo));
            if(!(c_pos.linear_velocity || c_pos.angular_velocity)) break;
        }
    }

    bool check_accur(const float& val1, const float& val2){
        return abs(val1) > abs(val2);
    }

    bool move_to_goal(char* argv[]){
        float x_target = atof(argv[1]);
        float y_target = atof(argv[2]);
        float theta = atof(argv[3]);

        turtlesim::msg::Pose c_pos;
        get_turtle_pos(&c_pos);

        RCLCPP_INFO(this->get_logger(), "Current position %f %f %f", c_pos.x, c_pos.y, c_pos.theta);
        RCLCPP_INFO(this->get_logger(), "Goal set to %f %f %f", x_target, y_target, theta);

        float x_delta = (x_target-c_pos.x);
        float y_delta = (y_target-c_pos.y);

        float distance_to_target = sqrtf(powf(x_delta, 2) + powf(y_delta, 2));
        float angle_to_goal      = atan2f(y_delta, x_delta) - c_pos.theta;

        if (check_accur(distance_to_target, DISTANCE_ACCURACY)) {
            if (check_accur(angle_to_goal, ANGLE_ACCURACY)) {
                RCLCPP_INFO(this->get_logger(), "[0]\tRotate %f for targeting ", angle_to_goal);
                send_message_sync(0, angle_to_goal);  // Rotate on angle to target
            }
            RCLCPP_INFO(this->get_logger(), "[1]\tMoving %f to goal", distance_to_target);
            send_message_sync(distance_to_target); // Move to target
        }

        get_turtle_pos(&c_pos);
        float target_angle = theta - c_pos.theta;

        if(check_accur(target_angle, ANGLE_ACCURACY)){
            RCLCPP_INFO(this->get_logger(), "[2]\tRotate %f for goal", target_angle);
            send_message_sync(0, target_angle); // Rotate to target angle
        }

        get_turtle_pos(&c_pos);
        RCLCPP_INFO(this->get_logger(), "End position %f %f %f", c_pos.x, c_pos.y, c_pos.theta);
    }

    turtlesim::msg::Pose::SharedPtr turtle_position;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;

};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    std::make_shared<MoveToGoal>(rclcpp::NodeOptions(), argc, argv);
    // rclcpp::spin(std::make_shared<MoveToGoal>(rclcpp::NodeOptions(), argc, argv));
    rclcpp::shutdown();
    return 0;
    

}