import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 参数定义
    ur_type = "ur5"
    pkg_name = "ur_app"
    description_file = "workcell.xacro"
    
    # 1. 启动 Gazebo 仿真
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_simulation_gazebo"), "/launch", "/ur_sim_control.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": "true",
            "runtime_config_package": pkg_name,
            "controllers_file": "sim_controllers.yaml",
            "description_package": pkg_name,
            "description_file": description_file,
            "launch_rviz": "false", 
            # --- 修改点 1：启动 joint_trajectory_controller ---
            "initial_joint_controller": "joint_trajectory_controller" 
        }.items(),
    )

    # 2. 启动 MoveIt
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_moveit_config"), "/launch", "/ur_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": "true",
            "description_package": pkg_name,
            "description_file": description_file,
            "moveit_config_package": "ur_moveit_config",
            "moveit_config_file": "ur.srdf.xacro",
            "use_sim_time": "true",
            "launch_rviz": "true",
            # --- 修改点 2：既然它想要 fake，就给它 true (默认也是true) ---
            "use_fake_hardware": "true", 
        }.items(),
    )

    return LaunchDescription([
        ur_control_launch,
        ur_moveit_launch
    ])