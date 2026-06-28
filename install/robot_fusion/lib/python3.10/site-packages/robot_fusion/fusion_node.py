import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        
        # 1. 全局影子状态存储 (初始化为空或默认值)
        self.world_state = {
            "handles": {
                "handle_left": {},
                "handle_mid": {},
                "handle_right": {}
            },
            "chassis": {},
            "vision": {
                "sequence": [0,0,0,0,0],
                "status": "idle"
            },
            "last_heartbeat": 0
        }

        # 2. 订阅 ProtocolNode 发布的所有物理状态
        # 你的 ProtocolNode 统一发布到 /robot_shadow_states
        self.create_subscription(String, '/robot_shadow_states', self.shadow_cb, 20)

        # 3. 订阅 VisionNode 发布的识别结果
        self.create_subscription(String, '/vision_detections', self.vision_cb, 10)

        # 4. 发布打包后的全局状态
        self.state_pub = self.create_publisher(String, '/world_state', 10)

        # 5. 定时发布器 (50Hz) 保证控制系统的高实时性
        self.create_timer(0.02, self.timer_cb)

        self.get_logger().info("融合层已对接 ProtocolNode 与 VisionNode。")

    # ======================== 回调处理 ========================

    def shadow_cb(self, msg):
        """
        解析 ProtocolNode 的输出：
        格式示例：{"source": "handle_left", "cmd": "0x64", "state": {...}}
        """
        try:
            data = json.loads(msg.data)
            source = data.get("source")
            state_data = data.get("state")

            if source in self.world_state["handles"]:
                # 更新左/中/右 Handle 状态
                self.world_state["handles"][source] = state_data
            elif source == "chassis":
                # 更新底盘状态
                self.world_state["chassis"] = state_data
            
            # 记录最新更新时间
            self.world_state["last_heartbeat"] = time.time()
        except Exception as e:
            self.get_logger().error(f"融合层解析影子状态失败: {e}")

    def vision_cb(self, msg):
        """
        解析 VisionNode 的输出：
        格式示例：{"sequence": [1,2,3,4,5], "mode": "Instant_Weighted_Fusion", ...}
        """
        try:
            data = json.loads(msg.data)
            self.world_state["vision"]["sequence"] = data["sequence"]
            self.world_state["vision"]["status"] = "confirmed"
            self.get_logger().info(f"融合层已锁定视觉序列: {data['sequence']}")
        except Exception as e:
            self.get_logger().error(f"融合层解析视觉数据失败: {e}")

    # ======================== 心跳发布 ========================

    def timer_cb(self):
        """以 50Hz 频率将打包好的全量数据发布出去"""
        msg = String()
        # 可以在这里增加系统运行时间戳
        output = {
            "timestamp": time.time(),
            "data": self.world_state
        }
        msg.data = json.dumps(output)
        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()