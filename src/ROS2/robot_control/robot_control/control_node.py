import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # ======================== 硬件配置表（由你定义） ========================
        # 请在下方 self.device_tree 和 self.cmd_map 中填入你的物理参数
        self.device_tree = {
            # TODO: 填入你的 handle_left, chassis 等映射
        }

        self.cmd_map = {
            # TODO: 填入 stepper, bldc, servo 等动作映射
        }

        # ======================== 消息接口 ========================
        # 订阅大脑发布的语义指令
        self.create_subscription(String, '/brain_cmd', self.brain_cb, 10)
        
        # 发布给解析层的底层指令
        self.control_pub = self.create_publisher(String, '/control_cmd', 10)
        
        self.get_logger().info("控制中台（Control Layer）框架启动成功。等待映射表注入...")

    def brain_cb(self, msg):
        """
        核心路由分发：
        大脑指令示例: {"device": "handle_left", "subsystem": "bldc", "action": "start", "params": {...}}
        """
        try:
            task = json.loads(msg.data)
            dev_name = task.get("device")
            sub_name = task.get("subsystem")
            action   = task.get("action")
            params   = task.get("params", {})

            # 1. 安全过滤：检查映射表是否已建立
            if not self.device_tree or not self.cmd_map:
                self.get_logger().warn("映射表为空，请先完善 device_tree 和 cmd_map!")
                return

            # 2. 路由匹配：寻找物理执行器
            device_info = self.device_tree.get(dev_name)
            if not device_info:
                self.get_logger().error(f"大脑指令错误：找不到大设备 [{dev_name}]")
                return

            sub_info = device_info.get(sub_name)
            if not sub_info:
                self.get_logger().error(f"大设备 [{dev_name}] 下找不到子执行器 [{sub_name}]")
                return

            # 3. 翻译转换：将语义 action 转换为底层 hex_cmd
            dev_type = sub_info.get("type")
            hex_cmd = self.cmd_map.get(dev_type, {}).get(action)

            if hex_cmd is None:
                self.get_logger().error(f"类型 [{dev_type}] 不支持动作 [{action}]")
                return

            # 4. 执行外发：发送给解析层 (ProtocolNode)
            self.dispatch_to_protocol(
                target=dev_name,    # 用于解析层判断发给哪个 TCP/串口
                sub_id=sub_info["sub_id"], 
                cmd_hex=hex_cmd, 
                params=params
            )

        except Exception as e:
            self.get_logger().error(f"控制层翻译异常: {e}")

    def dispatch_to_protocol(self, target, sub_id, cmd_hex, params):
        """
        封装为解析层可读的 JSON
        """
        payload = {
            "target": target,      # 目标物理链路
            "sub_id": sub_id,      # 执行器内部编号
            "cmd_hex": cmd_hex,    # 16进制指令码
            "params": params       # 具体参数 (如位置、速度值)
        }
        
        msg = String()
        msg.data = json.dumps(payload)
        self.control_pub.publish(msg)
        self.get_logger().info(f"OK -> [{target}] SubID:{sub_id} CMD:{hex(cmd_hex)}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ControlNode())
    rclpy.shutdown()