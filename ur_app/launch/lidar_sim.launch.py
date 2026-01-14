import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='ur_app').find('ur_app')
    xacro_file = os.path.join(pkg_share, 'urdf', 'hesai_xt16.xacro')

    # 解析 xacro
    robot_desc = Command(['xacro ', xacro_file])

    return LaunchDescription([
        # 1. 发布 TF 变换 (必须开启 use_sim_time)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='lidar_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True # <--- 之前可能就是缺了这个导致 RViz 报错
            }],
        ),
        # 2. 生成实体
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'hesai_stand'],
            output='screen'
        )
    ])