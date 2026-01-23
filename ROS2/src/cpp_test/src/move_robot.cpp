#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // 1. 初始化 ROS 2
  rclcpp::init(argc, argv);

  // 2. 创建节点
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // 创建一个后台执行器来处理回调
  auto const logger = rclcpp::get_logger("hello_moveit");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // 3. 创建 MoveGroupInterface (这是控制 Panda 的关键)
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // 4. 设置目标：使用 Panda 自带的 'ready' 姿态
  RCLCPP_INFO(logger, "📍 目标: 前往 'ready' 姿态");
  move_group_interface.setNamedTarget("ready");

  // 5. 规划与执行
  moveit::core::MoveItErrorCode result = move_group_interface.move();

  if (result == moveit::core::MoveItErrorCode::SUCCESS) {
     RCLCPP_INFO(logger, "✅ 运动执行成功！");
  } else {
     RCLCPP_ERROR(logger, "❌ 运动失败！");
  }

  // 6. 休息并关闭
  std::this_thread::sleep_for(std::chrono::seconds(2));
  rclcpp::shutdown();
  return 0;
}