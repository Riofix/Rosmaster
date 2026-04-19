import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')
        
        # ======================== 自动化状态定义 ========================
        self.ST_INIT       = 0  # 设备在线自检
        self.ST_WAIT_VIS   = 1  # 等待 1.5m 视觉识别序列
        self.ST_MOVE_WORK  = 2  # 前往作业区（同步对准）
        self.ST_GRABBING   = 3  # 执行垂直抓取循环（下降-抓-升）
        self.ST_MOVE_DROP  = 4  # 动态映射计算：前往放置区
        self.ST_UNLOADING  = 5  # 舵机卸料
        self.ST_FINISHED   = 6  # 任务完成
        
        self.state = self.ST_INIT
        self.world = None
        self.has_sent_cmd = False # 状态锁：确保每个状态只发一次任务指令
        
        # 数据存储
        self.target_seq = None    # 视觉识别结果，例如 [2, 1, 5, 4, 3]
        self.grab_colors = {}     # 抓取到的颜色，例如 {"handle_left": 1, ...}

        # ROS 接口
        self.create_subscription(String, '/world_state', self.world_cb, 10)
        self.brain_pub = self.create_publisher(String, '/brain_cmd', 10)
        
        # 决策频率：10Hz
        self.create_timer(0.1, self.state_machine_loop)
        self.get_logger().info("Decision Brain Node initialized. Starting Step 1: Init.")

    def world_cb(self, msg):
        """获取上帝视角数据"""
        try:
            self.world = json.loads(msg.data)["data"]
        except Exception as e:
            self.get_logger().error(f"WorldState parse error: {e}")

    def dispatch_task(self, device, subsystem, action, params=None):
        """
        三段式语义指令：
        device: 目标设备 (handle_left/mid/right, chassis)
        subsystem: 哪个子系统 (stepper_x, stepper_z, bldc, servo)
        action: 执行什么动作 (move_to, start_cycle, rotate)
        """
        cmd = {
            "device": device,
            "subsystem": subsystem,
            "action": action,
            "params": params if params else {}
        }
        self.brain_pub.publish(String(data=json.dumps(cmd)))
        self.has_sent_cmd = True 
        self.get_logger().info(f"Published Task: {device}-{subsystem} -> {action}")

    # ======================== 核心自动化状态机 ========================

    def state_machine_loop(self):
        if not self.world:
            return

        # --- 1. 初始化检查 ---
        if self.state == self.ST_INIT:
            # 检查影子系统是否已收录必要设备数据
            if self.world["chassis"] and self.world["handles"]["handle_left"]:
                self.get_logger().info("All devices online. Waiting for Vision results...")
                self.state = self.ST_WAIT_VIS

        # --- 2. 等待视觉 ---
        elif self.state == self.ST_WAIT_VIS:
            if self.world["vision"]["status"] == "confirmed":
                self.target_seq = self.world["vision"]["sequence"]
                self.get_logger().info(f"Target Sequence Confirmed: {self.target_seq}")
                self.state = self.ST_MOVE_WORK
                self.has_sent_cmd = False

        # --- 3. 前往作业区并对准 ---
        elif self.state == self.ST_MOVE_WORK:
            if not self.has_sent_cmd:
                # 给底盘下令前往 1.5m 处
                self.dispatch_task("chassis", "stepper_main", "move_to", {"pos": 1500})
                # 给三个抓手下令在横向轨道对齐
                for h in ["handle_left", "handle_mid", "handle_right"]:
                    self.dispatch_task(h, "stepper_x", "move_to", {"pos": 100})
            
            # 判断全设备到位
            ch_ok = self.world["chassis"].get("arrival_done", False)
            h_ok = all([self.world["handles"][h].get("track_arrived", False) for h in ["handle_left", "handle_mid", "handle_right"]])
            
            if ch_ok and h_ok:
                self.get_logger().info("Arrival WorkZone. Starting Grab Cycle.")
                self.state = self.ST_GRABBING
                self.has_sent_cmd = False

        # --- 4. 垂直抓取作业 ---
        elif self.state == self.ST_GRABBING:
            if not self.has_sent_cmd:
                # 触发下位机自动流程：下降 -> 无刷电机转动 -> 上升
                for h in ["handle_left", "handle_mid", "handle_right"]:
                    self.dispatch_task(h, "stepper_z", "start_grab_cycle", {"speed": 80})
            
            # 检查三个抓手是否都完成了垂直流程
            if all([self.world["handles"][h].get("action_done", False) for h in ["handle_left", "handle_mid", "handle_right"]]):
                # 核心步骤：抓取完成，立刻登记此时的颜色 ID
                for h in ["handle_left", "handle_mid", "handle_right"]:
                    self.grab_colors[h] = self.world["handles"][h].get("color_id")
                
                self.get_logger().info(f"Grab Task Finished. Colors registered: {self.grab_colors}")
                self.state = self.ST_MOVE_DROP
                self.has_sent_cmd = False

        # --- 5. 前往放置区（动态映射映射中心） ---
        elif self.state == self.ST_MOVE_DROP:
            if not self.has_sent_cmd:
                # =============================================================
                # 【动态目标映射逻辑修改区】
                # 此处你需要根据 self.target_seq (视觉) 和 self.grab_colors (实物)
                # 计算出底盘或抓手下一步要去的具体坐标。
                # =============================================================
                
                # 举例：计算结果映射表
                mapping_result = {}
                for hand, color in self.grab_colors.items():
                    if color in self.target_seq:
                        slot_index = self.target_seq.index(color) # 找到颜色对应的槽位(0-4)
                        mapping_result[hand] = slot_index
                
                # 发布带有计算结果的指令给控制层
                self.dispatch_task("chassis", "stepper_main", "move_to_placement", {
                    "mapping": mapping_result, 
                    "vision_info": self.target_seq
                })
                # =============================================================
            
            # 等待底盘带着轨道整体移动到放置位点
            if self.world["chassis"].get("arrival_done", False):
                self.get_logger().info("Arrived at Placement Zone. Dumping...")
                self.state = self.ST_UNLOADING
                self.has_sent_cmd = False

        # --- 6. 卸料倾倒 ---
        elif self.state == self.ST_UNLOADING:
            if not self.has_sent_cmd:
                # 驱动舵机执行 180 度翻转动作
                self.dispatch_task("chassis", "servo_dump", "rotate", {"angle": 180})
            
            # 简单逻辑：假设 2 秒后卸料完成，也可以根据反馈来
            time.sleep(2.0)
            self.state = self.ST_FINISHED
            self.has_sent_cmd = False

        # --- 7. 任务结束与归位 ---
        elif self.state == self.ST_FINISHED:
            if not self.has_sent_cmd:
                self.get_logger().info("Mission Success. Moving to Home position.")
                self.dispatch_task("chassis", "stepper_main", "move_to", {"pos": 0})
            
            # 执行完成后关闭节点
            if self.world["chassis"].get("arrival_done", False):
                self.get_logger().info("All steps completed. Brain shutting down.")
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(BrainNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()