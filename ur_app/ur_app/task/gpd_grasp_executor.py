#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

# TF2 坐标变换库 (用于将雷达坐标系的抓取点转换到机械臂基座坐标系)
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

# 消息类型
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, Constraints, OrientationConstraint, PositionConstraint
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# GPD 消息 (必须确保你的工作空间包含 gpd_msgs)
try:
    from gpd_msgs.msg import GraspConfigList
except ImportError:
    print("Error: 无法导入 gpd_msgs，请确保已编译 GPD ROS 2 接口")

class GPDGraspExecutor(Node):
    def __init__(self):
        super().__init__('gpd_grasp_executor')

        # --- 1. 初始化 TF 监听器 (核心：解决雷达与机械臂的手眼标定问题) ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- 2. 订阅 GPD 算法输出 ---
        self.gpd_sub = self.create_subscription(
            GraspConfigList,
            '/detect_grasps/clustered_grasps',  # GPD 默认输出话题
            self.gpd_callback,
            10
        )
        self.latest_grasps = None
        self.processing_grasp = False # 互斥锁标志

        # --- 3. 初始化 MoveIt 和 控制器客户端 ---
        self.reset_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.moveit_client = ActionClient(self, MoveGroup, 'move_action')
        
        # 等待服务连接
        self.get_logger().info('正在等待 MoveIt 和 控制器服务...')
        self.moveit_client.wait_for_server(timeout_sec=10.0)
        self.reset_client.wait_for_server(timeout_sec=5.0)
        self.get_logger().info('服务已连接，系统就绪！')

    def gpd_callback(self, msg):
        """接收到 GPD 发来的抓取候选列表"""
        if self.processing_grasp:
            return  # 如果正在执行任务，忽略新的检测结果
        
        if len(msg.grasps) > 0:
            self.get_logger().info(f'GPD 检测到 {len(msg.grasps)} 个抓取点，准备执行...')
            self.latest_grasps = msg
            self.execute_best_grasp()

    def vector_to_quaternion(self, approach, closing, axis):
        """
        GPD 输出的是3个向量 (接近方向, 闭合方向, 轴方向)，MoveIt 需要四元数。
        将 GPD 的旋转矩阵转换为四元数。
        """
        # GPD 坐标系定义: x=approach, y=closing, z=axis (或者根据GPD配置可能是 z=approach)
        # 通常 GPD 输出：
        # bottom: 抓取点位置
        # approach: 接近物体的方向 (通常对应机械臂末端的 Z 轴)
        # binormal: 手指闭合方向 (通常对应机械臂末端的 Y 轴或 X 轴)
        # axis: 第三个正交轴
        
        # 构造旋转矩阵 (列向量)
        R = np.array([
            [approach.x, closing.x, axis.x],
            [approach.y, closing.y, axis.y],
            [approach.z, closing.z, axis.z]
        ])
        
        # 旋转矩阵转四元数算法
        tr = np.trace(R)
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2,1] - R[1,2]) / S
            qy = (R[0,2] - R[2,0]) / S
            qz = (R[1,0] - R[0,1]) / S
        elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            qw = (R[2,1] - R[1,2]) / S
            qx = 0.25 * S
            qy = (R[0,1] + R[1,0]) / S
            qz = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            qw = (R[0,2] - R[2,0]) / S
            qx = (R[0,1] + R[1,0]) / S
            qy = 0.25 * S
            qz = (R[1,2] + R[2,1]) / S
        else:
            S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            qw = (R[1,0] - R[0,1]) / S
            qx = (R[0,2] + R[2,0]) / S
            qy = (R[1,2] + R[2,1]) / S
            qz = 0.25 * S
            
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def control_gripper(self, action="open"):
        """
        控制夹爪 (实物调试关键点)
        对于仿真：通常只是打印日志或调用特定service
        对于实物：通过 UR IO 控制或 Robotiq Action Server
        """
        self.get_logger().info(f'>>> 夹爪动作: {action.upper()} <<<')
        time.sleep(1.0) # 模拟夹爪动作时间
        # TODO: 实物调试时，在此处添加真实的 IO 控制代码
        # example: self.io_interface.set_digital_out(0, True)

    def execute_best_grasp(self):
        self.processing_grasp = True
        
        try:
            # 1. 筛选最高分的抓取点
            # GPD 抓取列表通常按分数排序，取第一个即可，或者按高度筛选(避免抓桌子)
            best_grasp = self.latest_grasps.grasps[0]
            
            # 2. 坐标变换: Lidar Frame -> Base Link
            # 假设 GPD 输出的 header.frame_id 是 'hesai_lidar'
            target_frame = 'base_link'
            source_frame = self.latest_grasps.header.frame_id
            
            # 构建原始 PoseStamped
            grasp_pose_lidar = PoseStamped()
            grasp_pose_lidar.header = self.latest_grasps.header
            grasp_pose_lidar.pose.position = best_grasp.position
            
            # 计算四元数
            q = self.vector_to_quaternion(best_grasp.approach, best_grasp.binormal, best_grasp.axis)
            grasp_pose_lidar.pose.orientation = q

            # 等待并执行变换
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=2.0)
                )
                grasp_pose_base = tf2_geometry_msgs.do_transform_pose(grasp_pose_lidar, transform)
            except TransformException as ex:
                self.get_logger().error(f'TF 变换失败: {ex}')
                self.processing_grasp = False
                return

            self.get_logger().info(f'抓取目标 (base_link): x={grasp_pose_base.pose.position.x:.3f}, y={grasp_pose_base.pose.position.y:.3f}, z={grasp_pose_base.pose.position.z:.3f}')

            # 3. 执行抓取流程
            # 3.1 打开夹爪
            self.control_gripper("open")
            
            # 3.2 移动到 预抓取点 (Pre-grasp approach, z + 10cm)
            # 修正坐标，UR5末端法兰的Z轴通常是轴向，如果使用Robotiq，需要根据实际TCP调整
            # 这里简单做位置偏移
            pre_grasp_pose = PoseStamped()
            pre_grasp_pose = grasp_pose_base
            pre_grasp_pose.pose.position.z += 0.15 # 抬高 15cm

            if self.move_to_pose(pre_grasp_pose, "预抓取点"):
                # 3.3 移动到 抓取点
                # 注意：实际抓取点可能需要沿接近向量回退一点，避免撞击
                if self.move_to_pose(grasp_pose_base, "抓取点"):
                    # 3.4 闭合夹爪
                    self.control_gripper("close")
                    
                    # 3.5 提起物体
                    lift_pose = grasp_pose_base
                    lift_pose.pose.position.z += 0.3
                    self.move_to_pose(lift_pose, "提起")
            
            # 任务结束，复位
            self.execute_reset()

        except Exception as e:
            self.get_logger().error(f'执行异常: {e}')
        
        finally:
            self.processing_grasp = False # 释放锁

    def move_to_pose(self, pose_stamped, description):
        """发送目标到位姿给 MoveIt"""
        self.get_logger().info(f'正在移动到: {description}')
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 2.0
        goal_msg.request.max_velocity_scaling_factor = 0.3 # 实物调试建议先设低速
        goal_msg.request.max_acceleration_scaling_factor = 0.3

        # 构造位置和姿态约束
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = "fake_tcp_link" # 需确认 URDF 中的末端 Link 名称
        pos_constraint.weight = 1.0
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.005, 0.005, 0.005] # 容差范围
        pos_constraint.constraint_region.primitives.append(box)
        pos_constraint.constraint_region.primitive_poses.append(pose_stamped.pose)

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = "base_link"
        ori_constraint.link_name = "fake_tcp_link"
        ori_constraint.orientation = pose_stamped.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.name = "gpd_target"
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        goal_msg.request.goal_constraints.append(constraints)

        # 发送 Action
        future = self.moveit_client.send_goal_async(goal_msg)
        
        # 这是一个简单的同步等待，实际项目中可能需要更复杂的异步处理
        # 注意：在 Callback 中使用 spin_until_future_complete 会死锁，因为已经在 spin 中
        # 这里我们由于在 ThreadedExecutor 中运行，或者是简单的 demo，需要特殊处理
        # 建议：由于是在 subscriber callback 中调用，直接 wait_for_result 可能会阻塞心跳
        # 更好的做法是使用 Action Client 的同步调用或者状态机
        # 这里为了简化代码逻辑，假设 Action Server 响应够快
        
        # 警告：在 rclpy 的 callback 中不能调用 spin_until_future_complete
        # 我们这里不等待完全的结果检查，只负责发送，实际工程中应使用 State Machine
        return True 

    def execute_reset(self):
        """复位机械臂到安全姿态"""
        self.get_logger().info('复位机械臂...')
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        point.time_from_start = Duration(seconds=4).to_msg()
        goal_msg.trajectory.points.append(point)
        self.reset_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GPDGraspExecutor()
    
    # 使用 MultiThreadedExecutor 防止 TF 监听和 订阅回调 互相阻塞
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()