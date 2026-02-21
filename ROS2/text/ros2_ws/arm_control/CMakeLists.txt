cmake_minimum_required(VERSION 3.8)
project(arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# ================= 就是少了下面这极其关键的几行！ =================
# 1. 告诉系统要把 src/motor_controller.cpp 编译成叫 motor_controller 的程序
add_executable(motor_controller src/motor_controller.cpp)

# 2. 告诉系统这个程序需要用到 rclcpp 和 sensor_msgs 这两个库
ament_target_dependencies(motor_controller rclcpp sensor_msgs)

# 3. 把编译好的程序安装到 ROS2 能找到的地方
install(TARGETS
        motor_controller
        DESTINATION lib/${PROJECT_NAME})
# ================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()