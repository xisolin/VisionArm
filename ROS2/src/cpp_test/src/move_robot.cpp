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
    auto move_group_interface = MoveGroupInterface(node,"panda_arm");

    move_group_interface.setMaxVelocityScalingFactor(0.3);
    move_group_interface.setMaxAccelerationScalingFactor(0.3);

    RCLCPP_INFO(logger, "正在准备连续路经...");

    std::vector<geometry_msgs::msg::Pose> waypoints;

    geometry_msgs::msg::Pose start_pose = move_group_interface.getCurrentPose().pose;

    waypoints.push_back(start_pose);

    geometry_msgs::msg::Pose target_pose = start_pose;

    //向左
    target_pose.position.y += 0.2;
    waypoints.push_back(target_pose);

    //向上
    target_pose.position.z += 0.2;
    waypoints.push_back(target_pose);

    //向右
    target_pose.position.y -= 0.2;
    waypoints.push_back(target_pose);

    //向下
    target_pose.position.z -= 0.2;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    RCLCPP_INFO(logger,"正在计算平滑路径");
    double fraction = move_group_interface.computeCartesianPath(
        waypoints,eef_step,jump_threshold,trajectory);

    RCLCPP_INFO(logger,"路径规划覆盖率:%.2f%%",fraction * 100.0);

    if (fraction > 0.9) {
        RCLCPP_INFO(logger,"路径完美！正在执行连续运动");

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        my_plan.trajectory = trajectory;

        move_group_interface.execute(my_plan);
        RCLCPP_INFO(logger,"绘制完成");
    }else {
        RCLCPP_INFO(logger,"规划失败");
    }

    rclcpp::shutdown();

    return 0;
}