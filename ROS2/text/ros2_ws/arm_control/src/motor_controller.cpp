#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>

class MotorController : public rclcpp::Node {
public:
    MotorController() : Node("motor_controller") {
        //创建发布者
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
        
    }
private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
};