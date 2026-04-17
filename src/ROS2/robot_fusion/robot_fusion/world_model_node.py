#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import WorldState, CraneState, DetectionList
from nav_msgs.msg import Odometry

class WorldModelNode(Node):
    """
    负责收集机器人全身所有的感知与状态数据
    并对齐汇聚为唯一的 WorldState (世界状态黑板)，供大脑层决策寻址。
    """
    def __init__(self):
        super().__init__('world_model_node')
        
        self.world_state = WorldState()
        self.world_state.current_state = WorldState.SYSTEM_IDLE
        
        # 1. 订阅底层物理解析器扔上来的三台吊机数据
        self.create_subscription(CraneState, '/crane_left/state', lambda m: self.crane_cb('crane_left', m), 10)
        self.create_subscription(CraneState, '/crane_middle/state', lambda m: self.crane_cb('crane_middle', m), 10)
        self.create_subscription(CraneState, '/crane_right/state', lambda m: self.crane_cb('crane_right', m), 10)
        
        # 2. 订阅视觉层的感知结果
        self.create_subscription(DetectionList, '/vision/detections', self.vision_cb, 10)
        
        # 3. 订阅底盘里程计 (若存在)
        self.create_subscription(Odometry, '/chassis/odom', self.odom_cb, 10)
        
        # 对外全网广播权威的整合状态
        self.state_pub = self.create_publisher(WorldState, '/world_state', 10)
        
        # 固定 10Hz 高频广播，确保决策层数据随时新鲜
        self.timer = self.create_timer(0.1, self.publish_world_state)
        self.get_logger().info("World Model 数据融合中心上线运转。")

    def crane_cb(self, crane_id, msg: CraneState):
        """记录对应的分布式吊机姿态与状态"""
        if crane_id == 'crane_left':
            self.world_state.crane_left = msg
        elif crane_id == 'crane_middle':
            self.world_state.crane_middle = msg
        elif crane_id == 'crane_right':
            self.world_state.crane_right = msg

    def vision_cb(self, msg: DetectionList):
        """记录最新的数字视觉识别与像素坐标数据"""
        self.world_state.visible_targets = msg

    def odom_cb(self, msg: Odometry):
        """假装同步底盘数据"""
        self.world_state.chassis_x = msg.pose.pose.position.x
        self.world_state.chassis_y = msg.pose.pose.position.y

    def publish_world_state(self):
        """添加时间戳后抛出全系统快照"""
        self.world_state.header.stamp = self.get_clock().now().to_msg()
        self.world_state.header.frame_id = "world_map"
        self.state_pub.publish(self.world_state)

def main(args=None):
    rclpy.init(args=args)
    node = WorldModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
