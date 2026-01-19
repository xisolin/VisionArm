#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import sys

# 引入 MoveIt 2 的 Python 接口
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder


def main():
    # 1. 初始化 ROS 2
    rclpy.init()

    # ---------------------------------------------------------
    # ⚠️ 配置区 (已修正为官方包名)
    # ---------------------------------------------------------
    # 因为刚才我们手动修复了源码里的 srdf，所以直接用官方包名
    NEW_PACKAGE_NAME = "annin_ar4_moveit_config"

    # 机器人名字
    ROBOT_NAME = "annin_ar4"

    # 规划组名字
    GROUP_NAME = "manipulator"
    # ---------------------------------------------------------

    print(f"🔄 正在加载配置包: {NEW_PACKAGE_NAME} ...")

    try:
        # 2. 加载机械臂配置
        moveit_config = (
            MoveItConfigsBuilder(ROBOT_NAME, package_name=NEW_PACKAGE_NAME)
            .to_moveit_configs()
        )
    except Exception as e:
        print("\n❌ 配置加载失败！")
        print("\n❌ 配置加载失败！")
        print(f"错误信息: {e}")
        return

    # 3. 创建 MoveItPy 对象
    try:
        ar4_robot = MoveItPy(ROBOT_NAME, config_dict=moveit_config.to_dict())
    except Exception as e:
        print(f"\n❌ 初始化机器人失败: {e}")
        return

    # 4. 获取规划组
    try:
        arm = ar4_robot.get_planning_component(GROUP_NAME)
    except Exception as e:
        print(f"\n❌ 找不到规划组 '{GROUP_NAME}'")
        return

    print("\n🤖 机械臂连接成功！准备运动...")

    # 5. 设置目标
    print("📍 目标: 回到 'home' 姿态")
    arm.set_start_state_to_current_state()

    # 尝试设置 home，如果没有就设为 random
    try:
        arm.set_goal_state(configuration_name="home")
    except Exception:
        print("⚠️ 警告: 没找到 'home' 姿态，尝试随机动作...")
        arm.set_goal_state(configuration_name="random")

        # 6. 规划与执行
    print("🧠 正在规划路径...")
    plan_result = arm.plan()

    if plan_result:
        print("✅ 规划成功！开始执行 (请看 Rviz)...")
        arm.execute()
    else:
        print("❌ 规划失败！")

    print("😴 任务完成")
    time.sleep(2)
    rclpy.shutdown()


if __name__ == "__main__":
    main()