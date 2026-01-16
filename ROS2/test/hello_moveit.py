
import time
import rclpy
from rclpy.node import Node

#这是MoveIt2的python接口
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

def main():
    rclpy.init()


    try:
        vision_arm = MoveItPy(node_name="moveit_python_demo")
    except Exception as e:
        print("初始化失败，请确保你已经source了环境并且有正确配置。")
        print("初始化失败，请确保你已经source了环境并且有正确配置。")
        print("初始化失败，请确保你已经source了环境并且有正确配置。")
        print("初始化失败，请确保你已经source了环境并且有正确配置。")
        print(e)
        return
    group_name = "manipulator"
    try:
        arm = vision_arm.get_planning_component(group_name)
    except Exception as e:
        print(f"找不到组名'{group_name}',请检查Rviz里的名字！")
        print("初始化失败，请确保你已经source了环境并且有正确配置。")
        return

    print("===机械臂已就绪，准备运动===")

    arm.set_goal_state(configuration_name="home")
    print("正在规划路径（回零）...")
    plan_result = arm.plan()

    if plan_result:
        print("规划成功，开始执行！")
        print("规划成功，开始执行！")
        print("规划成功，开始执行！")
        print("规划成功，开始执行！")
        print("规划成功，开始执行！")
        print("规划成功，开始执行！")
        print("规划成功，开始执行！")
        vision_arm.execute(plan_result.trajectory,controllers=[])
    else :
        print("规划失败")

    time.sleep(2)

    print("正在规划路径(随机姿态)...")

    arm.set_start_state_to_current_state()
    current_state = vision_arm.get_current_state()
    print("当前关节角度:",current_state.joint_positions)

    rclpy.shutdown()

    if __name__ == "__main__":
        main()