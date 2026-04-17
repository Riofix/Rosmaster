#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from robot_interfaces.action import CraneMovement
from robot_interfaces.msg import WorldState

class BrainNode(Node):
    """
    终极中枢节点 (最高控制权)
    基于世界模型 (WorldState) 的唯一真相进行统筹调度。
    它负责做最高级的物流任务调度，调用 dispatcher 的 Action。
    """
    def __init__(self):
        super().__init__('brain_node')
        
        # 订阅上帝视角的唯一的黑板数据
        self.create_subscription(WorldState, '/world_state', self.world_state_cb, 10)
        self.world = None

        # 连接 Dispatcher 的原生动作服务端 (代表大脑拥有下达高级手令的权力)
        self._action_client_left = ActionClient(self, CraneMovement, '/crane_left/movement')
        
        # 预留给未来挂载 Behavior Tree 或多线程调度
        # self.timer = self.create_timer(1.0, self.tick_bt)
        
        self.get_logger().info("====================================")
        self.get_logger().info("  物流大脑中枢 (Brain Layer) 正式上线  ")
        self.get_logger().info("====================================")

    def world_state_cb(self, msg: WorldState):
        self.world = msg

    def grab_object(self, crane_id, target_depth):
        """示例宏观动作：高阶包裹抓取逻辑下发"""
        self.get_logger().info(f"决策引擎下达命令 => [{crane_id}] 下探抓取深度: {target_depth}")
        
        self._action_client_left.wait_for_server()
        goal_msg = CraneMovement.Goal()
        goal_msg.axis = CraneMovement.Goal.TARGET_HOIST 
        goal_msg.target_pos = target_depth
        
        # 触发异步跟踪闭环
        self._action_client_left.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
