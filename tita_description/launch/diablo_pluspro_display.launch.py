'''
Author: hilab-workshop-ldc 2482812356@qq.com
Date: 2025-03-24 16:18:39
LastEditors: hilab-workshop-ldc 2482812356@qq.com
LastEditTime: 2025-06-06 17:11:35
FilePath: /tita_rl_sim2sim2real/src/tita_locomotion/tita_description/launch/diablo_pluspro_display.launch.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_tower",
            default_value="false",
            description="Decide use tita tower or not",
        )
    )
    use_tower = LaunchConfiguration("use_tower")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("tita_description"),
                    "diablo_plus_pro",
                    "xacro/display.xacro",
                ]
            ),
            " ",
            "use_tower:=",
            use_tower,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("tita_description"), "rviz", "diablo_pluspro.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    nodes = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]
    return LaunchDescription(declared_arguments + nodes)
