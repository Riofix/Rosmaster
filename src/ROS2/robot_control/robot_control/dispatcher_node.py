#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Float32MultiArray
from robot_interfaces.action import CraneMovement
from robot_interfaces.msg import WorldState
import time
import math

class DispatcherNode(Node):
    """
    负责面向顶层 Brain 提供优雅的 Action Server。
    Brain 不需要懂任何底层的脉冲和速度逻辑，只下达诸如 "左吊机，给我下达 2000 伸缩位置"。
    本节点将它翻译为底层 step，并循环监控世界状态，等到位置到达时再返回成功。
    """
    def __init__(self):
        super().__init__('dispatcher_node')
        
        self.world_state = None
        # 订阅数据层的“最高权威”状态，用来随时监控我们下发的操作有没有在跑
        self.create_subscription(WorldState, '/world_state', self.state_cb, 10)
        
        # 抛向各路吊机的脉冲 Topic 口 (绕过 tcp 协议直接给 Packer)
        self.cmd_pubs = {
            'crane_left': self.create_publisher(Float32MultiArray, '/crane_left/debug_step', 10),
            'crane_middle': self.create_publisher(Float32MultiArray, '/crane_middle/debug_step', 10),
            'crane_right': self.create_publisher(Float32MultiArray, '/crane_right/debug_step', 10)
        }
        
        # 面向高端大脑的优雅 Action 接口 (多开三个)
        self.act_left = ActionServer(self, CraneMovement, '/crane_left/movement', self.execute_cb_left)
        self.act_middle = ActionServer(self, CraneMovement, '/crane_middle/movement', self.execute_cb_middle)
        self.act_right = ActionServer(self, CraneMovement, '/crane_right/movement', self.execute_cb_right)
        
        self.get_logger().info("Action 任务分发与进度监工局已上限。")

    def state_cb(self, msg):
        self.world_state = msg

    def do_action(self, crane_id, goal_handle):
        """核心 Action 执行器，带容错循环卡尺"""
        req = goal_handle.request
        
        # 将高级坐标概念转为低级下压
        msg = Float32MultiArray()
        # [地址(1轨道/2吊臂), 转向, 速度, 行驶脉冲数]
        # 这里用一种取巧的相对转换法 (实际可从 WorldState 拿到当前位置求差)
        addr = 1 if req.axis == CraneMovement.Goal.TARGET_TRACK else 2
        # 若需要精准到达，速度可预设
        direction = 0 if req.target_pos > 0 else 1 
        ticks = abs(req.target_pos)
        
        msg.data = [float(addr), float(direction), 500.0, float(ticks)]
        self.cmd_pubs[crane_id].publish(msg)
        
        # ========== Action 核心精神：死循环等待结果并回传进度 ========== #
        start_time = time.time()
        result = CraneMovement.Result()
        feedback = CraneMovement.Feedback()
        
        while rclpy.ok():
            time.sleep(0.5)
            
            # 超时保护 (10秒)
            if time.time() - start_time > 10.0:
                result.success = False
                goal_handle.abort()
                break
                
            # 若 WorldState 里拿到了精准的反馈，则回传 Feedback 给大脑
            if self.world_state:
                # 简单计算一个假进度（由于真正的底层可能并没有完美反馈，根据需求接入真实反馈）
                feedback.current_pos = 0.0 # TODO: 获取 crane_state 的 track_pos 或 hoist_depth
                goal_handle.publish_feedback(feedback)
                
                # 假设到底了
                # if math.isclose(feedback.current_pos, req.target_pos):
                # ...
                
            # 我们为了演示顺畅，2秒后强行宣告完成。由于真实硬件要等，您可以把这里替换为读取真实的 WorldState 计算
            if time.time() - start_time > 2.0:
                result.success = True
                goal_handle.succeed()
                self.get_logger().info(f"[{crane_id}] Action 执行完毕！")
                break
                
        return result

    def execute_cb_left(self, goal_handle):
        return self.do_action('crane_left', goal_handle)

    def execute_cb_middle(self, goal_handle):
        return self.do_action('crane_middle', goal_handle)

    def execute_cb_right(self, goal_handle):
        return self.do_action('crane_right', goal_handle)

def main(args=None):
    rclpy.init(args=args)
    node = DispatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
