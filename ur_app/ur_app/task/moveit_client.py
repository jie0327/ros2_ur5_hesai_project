#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

# 消息类型导入
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    PositionConstraint,
    OrientationConstraint
)

class FullTaskExecutor(Node):
    def __init__(self):
        super().__init__('full_task_executor')
        
        # --- 修改点：连接到 joint_trajectory_controller ---
        self.reset_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # ... (MoveIt client 保持不变，因为它内部会自动找 joint_...)
        self.moveit_client = ActionClient(
            self, 
            MoveGroup, 
            'move_action'
        )

        self.get_logger().info('正在连接所有服务...')
        
        if not self.reset_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('底层控制器连接失败！')
            raise RuntimeError("Controller not available")
            
        if not self.moveit_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveIt 连接失败！')
            raise RuntimeError("MoveIt not available")
            
        self.get_logger().info('>>> 所有服务连接成功！准备执行任务。 <<<')

    def execute_reset(self):
        """执行复位动作：将机械臂移动到绝对安全的垂直姿态"""
        self.get_logger().info('--- [阶段 1/2] 开始复位机械臂 (垂直向上) ---')
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        point = JointTrajectoryPoint()
        
        # ==========================================
        # 修正点：使用垂直向上姿态，避免撞到雷达或自身
        # ==========================================
        point.positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        point.time_from_start = Duration(seconds=4).to_msg()
        
        goal_msg.trajectory.points.append(point)

        future = self.reset_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('复位请求被拒绝！')
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result()
        
        if result.result.error_code == 0:
            self.get_logger().info('复位成功！')
            return True
        else:
            self.get_logger().error(f'复位失败，错误码: {result.result.error_code}')
            return False

    def execute_moveit_plan(self, x, y, z):
        """执行 MoveIt 规划"""
        self.get_logger().info(f'--- [阶段 2/2] 开始 MoveIt 规划 -> 目标: [{x}, {y}, {z}] ---')

        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5

        # 位置约束
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = "fake_tcp_link" # 确保这里与你的 xacro 一致
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]
        
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position = Point(x=float(x), y=float(y), z=float(z))
        
        pos_constraint.constraint_region.primitives.append(box)
        pos_constraint.constraint_region.primitive_poses.append(box_pose.pose)
        pos_constraint.weight = 1.0

        # 姿态约束 (垂直向下)
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = "base_link"
        ori_constraint.link_name = "fake_tcp_link"
        ori_constraint.orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 3.14
        ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.name = "grasp_target"
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        goal_msg.request.goal_constraints.append(constraints)
        
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True

        future = self.moveit_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('MoveIt 拒绝了目标！(可能不可达)')
            return False

        self.get_logger().info('MoveIt 已接收请求，正在规划...')
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result()
        
        if result.result.error_code.val == 1:
            self.get_logger().info('>>> 任务圆满完成！机械臂已到达目标。 <<<')
            return True
        else:
            self.get_logger().error(f'MoveIt 执行失败，错误码: {result.result.error_code.val}')
            return False

def main(args=None):
    rclpy.init(args=args)
    try:
        executor = FullTaskExecutor()
        
        # 1. 先复位到垂直姿态 (解除碰撞)
        success = executor.execute_reset()
        if not success:
            return

        time.sleep(1.0) # 等待状态同步
        
        # 2. 再执行抓取移动
        # 注意：如果 z=0.3 离桌面太近导致 MoveIt 认为碰撞，可以尝试改高一点，比如 0.4
        executor.execute_moveit_plan(0.6, 0.0, 0.4)
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()