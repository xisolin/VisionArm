cmake_minimum_required(VERSION 3.8)
project(cpp_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找依赖库
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# 2. 编译可执行文件
add_executable(move_robot src/move_robot.cpp)

# 3. 链接库文件
ament_target_dependencies(move_robot
        rclcpp
        moveit_ros_planning_interface
)

# 4. 安装规则
install(TARGETS
        move_robot
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()