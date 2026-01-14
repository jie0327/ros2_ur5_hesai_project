#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import random
import math
import time

# 定义红色小方块模型 (5cm x 5cm x 5cm)
BOX_SDF = """
<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <pose>{x} {y} 0.025 0 0 {yaw}</pose>
    <link name='link'>
      <inertial><mass>0.1</mass></inertial>
      <collision name='collision'>
        <geometry><box><size>0.1 0.1 0.1</size></box></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
      <visual name='visual'>
        <geometry><box><size>0.1 0.1 0.1</size></box></geometry>
        <material>
          <script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Red</name></script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

class SmartSpawner(Node):
    def __init__(self):
        super().__init__('smart_spawner')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 Gazebo 服务...')
        
        # === 配置区域 ===
        # 雷达位置 (必须与 workcell.xacro 一致)
        self.lidar_pos = {'x': 0.8, 'y': 0.0}
        self.lidar_min_dist = 0.3  # 物体必须离雷达至少 0.3米 (防止盲区)
        
        # 机械臂抓取范围 (扇形区域)
        self.arm_min_reach = 0.35  # 太近抓不到
        self.arm_max_reach = 0.75  # 太远抓不到

    def is_valid_pos(self, x, y):
        """检查位置是否合法"""
        # 1. 检查是否在机械臂抓取范围内
        dist_to_arm = math.sqrt(x**2 + y**2)
        if dist_to_arm < self.arm_min_reach or dist_to_arm > self.arm_max_reach:
            return False
            
        # 2. 检查是否离雷达太近 (盲区保护)
        dist_to_lidar = math.sqrt((x - self.lidar_pos['x'])**2 + (y - self.lidar_pos['y'])**2)
        if dist_to_lidar < self.lidar_min_dist:
            return False
            
        return True

    def spawn_objects(self, count=5):
        spawned = 0
        attempts = 0
        
        while spawned < count and attempts < 100:
            attempts += 1
            
            # 随机生成坐标 (在机械臂前方半圆区域)
            angle = random.uniform(-1.57, 1.57) # -90度 到 +90度
            radius = random.uniform(self.arm_min_reach, self.arm_max_reach)
            
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            yaw = random.uniform(0, 3.14)

            # 验证坐标
            if self.is_valid_pos(x, y):
                name = f"box_{random.randint(1000,9999)}"
                self.spawn_one(name, x, y, yaw)
                spawned += 1
                time.sleep(0.2)
                
        self.get_logger().info(f"成功生成 {spawned} 个物体")

    def spawn_one(self, name, x, y, yaw):
        req = SpawnEntity.Request()
        req.name = name
        req.xml = BOX_SDF.format(name=name, x=x, y=y, yaw=yaw)
        req.reference_frame = "world"
        self.client.call_async(req)
        self.get_logger().info(f"生成: {name} at [{x:.2f}, {y:.2f}] (安全位置)")

def main(args=None):
    rclpy.init(args=args)
    spawner = SmartSpawner()
    spawner.spawn_objects(count=6) # 这里设置生成几个
    rclpy.shutdown()

if __name__ == '__main__':
    main()