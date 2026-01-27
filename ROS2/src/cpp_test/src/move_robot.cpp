#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs//msg/pose.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc,argv);
    auto const node = std::make_shared<rclcpp::Node>("draw_rectangle");
    auto const logger = rclcpp::get_logger("draw_rectangle");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor](){executor.spin();}).detach();

    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node,"panda_arm");\

    move_group_interface.setMaxVelocityScalingFactor(0.3);
    move_group_interface.setMaxAccelerationScalingFactor(0.3);

    RCLCPP_INFO(logger,"正在前往起点");
    move_group_interface.setNamedTarget("ready");
    move_group_interface.move();

    //获取当前姿态为基准
    geometry_msgs::msg::Pose target_pose = move_group_interface.getCurrentPose().pose;

    target_pose.position.x += 0.1;
    move_group_interface.setPoseTarget(target_pose);
    move_group_interface.move();

    target_pose = move_group_interface.getCurrentPose().pose;

    //向左
    RCLCPP_INFO(logger,"向左画");
    target_pose.position.y += 0.2;

    move_group_interface.setPoseTarget( target_pose);
    move_group_interface.move();

    //向上
    RCLCPP_INFO(logger,"向上画");
    target_pose.position.z += 0.2;

    move_group_interface.setPoseTarget(target_pose);
    move_group_interface.move();

    //向右
    RCLCPP_INFO(logger,"向右");
    target_pose.position.y -= 0.2;

    move_group_interface.setPoseTarget(target_pose);
    move_group_interface.move();

    //向下
    RCLCPP_INFO(logger,"向下");
    target_pose.position.z -= 0.2;

    move_group_interface.setPoseTarget(target_pose);
    move_group_interface.move();

    RCLCPP_INFO(logger,"矩形绘制完成");

    rclcpp::shutdown();

    return 0;
}