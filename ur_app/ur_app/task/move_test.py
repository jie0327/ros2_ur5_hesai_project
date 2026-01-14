#!/usr/bin/env python3
#ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py ur_type:=ur5 launch_rviz:=true
#ros2 launch ur_app lidar_sim.launch.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive

class SimpleMover(Node):
    def __init__(self):
        super().__init__('move_test')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self._action_client.wait_for_server()
        self.get_logger().info('MoveIt Connected!')

    def move_to_pose(self, x, y, z):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.num_planning_attempts = 20            #尝试次数
        goal_msg.request.allowed_planning_time = 10.0          #运动时间
        goal_msg.request.max_velocity_scaling_factor = 0.2
        goal_msg.request.max_acceleration_scaling_factor = 0.2

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = "tool0"
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.02, 0.02, 0.02] # 宽松公差
        
        pose_in_box = Pose()
        pose_in_box.position = Point(x=float(x), y=float(y), z=float(z))
        
        pos_constraint.constraint_region.primitives.append(box)
        pos_constraint.constraint_region.primitive_poses.append(pose_in_box)
        pos_constraint.weight = 1.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = "base_link"
        ori_constraint.link_name = "tool0"
        ori_constraint.orientation.x = 1.0 # 垂直向下
        ori_constraint.orientation.y = 0.0
        ori_constraint.orientation.z = 0.0
        ori_constraint.orientation.w = 0.0
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        goal_msg.request.goal_constraints.append(constraints)

        self.get_logger().info(f"Going to [{x}, {y}, {z}]...")
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal Rejected!')
            return

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        if res_future.result().result.error_code.val == 1:
            self.get_logger().info('Success!')
        else:
            self.get_logger().error('Failed!')

def main(args=None):
    rclpy.init(args=args)
    SimpleMover().move_to_pose(0.4, 0.2, 0.4)
    rclpy.shutdown()

if __name__ == '__main__':
    main()