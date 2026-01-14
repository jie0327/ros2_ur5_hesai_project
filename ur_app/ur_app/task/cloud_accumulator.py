#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Header

class CloudAccumulator(Node):
    def __init__(self):
        super().__init__('cloud_accumulator')
        
        # 1. 订阅原始稀疏点云
        self.sub = self.create_subscription(
            PointCloud2, 
            '/hesai/pandar', 
            self.callback, 
            10
        )
        
        # 2. 发布致密点云 (GPD监听这个话题)
        self.pub = self.create_publisher(PointCloud2, '/hesai/pandar_accumulated', 10)
        
        # 缓冲池
        self.point_buffer = []
        self.buffer_size = 30  # 累积 30 帧 (约3秒)，数值越大越密，但延迟越高
        
        self.get_logger().info("点云累积器已启动... 正在将16线雷达数据由稀疏变致密")

    def callback(self, msg):
        # 将 ROS 消息转为 Numpy (x, y, z)
        # skip_nans=True 过滤无效点
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        
        # 加入缓存
        self.point_buffer.extend(points)
        
        # 控制缓存大小，去掉最老的点 (滑动窗口)
        # 粗略估算：每帧约 2000 点，30帧约 60000 点
        max_points = 30 * 2000 
        if len(self.point_buffer) > max_points:
            self.point_buffer = self.point_buffer[-max_points:]

        # 每收到 5 帧发布一次，降低 CPU 负载
        if len(self.point_buffer) > 0:
            self.publish_cloud(msg.header)

    def publish_cloud(self, header):
        # 创建新的 PointCloud2
        # 注意：这里我们沿用最新一帧的 frame_id (通常是 hesai_lidar)
        dense_cloud = pc2.create_cloud_xyz32(header, self.point_buffer)
        self.pub.publish(dense_cloud)

def main(args=None):
    rclpy.init(args=args)
    node = CloudAccumulator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()