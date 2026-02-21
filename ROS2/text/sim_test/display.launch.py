from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # 1. 定义你的文件绝对路径（刚刚存的那些文件）
    urdf_file = '/home/max/github/VisionArm/ROS2/text/sim_test/single_motor.urdf'
    rviz_config = '/home/max/github/VisionArm/ROS2/text/sim_test/motor.rviz'

    # 2. 返回要同时启动的三个节点
    return LaunchDescription([
        # 节点 1：发布机器人模型 (相当于你之前敲的第一个终端)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
        ),

        # 节点 2：启动带滑动条的控制面板 (相当于第二个终端)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),

        # 节点 3：启动 RViz2，并自动加载你刚才保存的存档 (相当于第三个终端)
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config] # -d 就是加载配置文件的意思
        )
    ])