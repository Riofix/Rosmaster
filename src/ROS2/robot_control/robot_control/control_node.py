import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

# ================= 1. 移植自 Demo 的工业级 PID 控制器 =================
class PositionPID:
    def __init__(self, kp, ki, kd, max_out, max_i, max_accel=1000.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.max_i = max_i
        self.max_accel = max_accel 
        
        self.target = 0.0
        self.last_output = 0.0 
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.error = 0.0

    def compute(self, current_pos):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0.0: dt = 0.04 # 匹配底盘 40ms 周期

        self.error = self.target - current_pos

        # PID 核心计算
        p_out = self.kp * self.error
        self.integral += self.error * dt
        # 积分限幅 (Anti-Windup)
        self.integral = max(min(self.integral, self.max_i), -self.max_i)
        i_out = self.ki * self.integral
        d_out = self.kd * ((self.error - self.last_error) / dt)

        raw_output = p_out + i_out + d_out
        # 输出限幅
        raw_output = max(min(raw_output, self.max_out), -self.max_out)

        # 核心：加速度斜坡限制
        max_change = self.max_accel * dt 
        if raw_output > self.last_output + max_change:
            output = self.last_output + max_change
        elif raw_output < self.last_output - max_change:
            output = self.last_output - max_change
        else:
            output = raw_output

        self.last_output = output
        self.last_error = self.error
        self.last_time = now
        return output

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_output = 0.0
        self.last_time = time.time()

# ================= 2. 控制中台节点 =================
class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # --- 硬件映射配置 ---
        self.device_tree = {
            "chassis": {"base": {"sub_id": 0x01, "type": "mecanum_base"}},
            "handle_left": {
                "track": {"sub_id": 0x01, "type": "handle_servo"},
                "lift":  {"sub_id": 0x02, "type": "handle_servo"},
                "grab":  {"sub_id": 0x03, "type": "handle_bldc"}
            }
            # 其他抓手以此类推...
        }

        # 指令码定义
        self.cmd_map = {
            "mecanum_base": {"set_speed": 0x12},
            "handle_servo": {"move_to": 0x01},
            "handle_bldc":  {"start": 0x10}
        }

        # --- PID 实例初始化 (针对底盘) ---
        # 参数建议：Kp=0.28, Ki=0.08, Kd=0.1, MaxAccel=1000 (根据你demo)
        self.chassis_pid = PositionPID(kp=0.28, ki=0.08, kd=0.1, max_out=600, max_i=200, max_accel=1200.0)
        
        # 任务状态管理
        self.current_chassis_task_id = 0
        self.is_chassis_moving = False

        # --- ROS2 接口 ---
        # 订阅大脑指令
        self.create_subscription(String, '/brain_cmd', self.brain_cb, 10)
        # 订阅解析层影子状态 (用于 PID 反馈)
        self.create_subscription(String, '/robot_shadow_states', self.shadow_cb, 10)
        
        # 发布给 ProtocolPackNode (二进制封包)
        self.packer_pub = self.create_publisher(String, '/control_cmd', 10)
        # 发布给 ProtocolNode (内部状态强制更新)
        self.internal_state_pub = self.create_publisher(String, '/protocol_internal_cmd', 10)

        self.get_logger().info("ControlNode (PID + TaskID) 启动成功，底盘闭环已就绪。")

    # ================= 核心逻辑：大脑指令处理 =================
    def brain_cb(self, msg):
        try:
            task = json.loads(msg.data)
            device = task.get("device")
            action = task.get("action")
            task_id = task.get("task_id", 0)
            params = task.get("params", {})

            if device == "chassis":
                # 1. 重置影子状态 (TaskID 握手第一步)
                self.current_chassis_task_id = task_id
                self.notify_protocol_reset("chassis", task_id)
                
                # 2. 启动 PID 目标
                if action == "move_to":
                    target_pos = params.get("pos", 0)
                    self.chassis_pid.target = float(target_pos)
                    self.chassis_pid.reset()
                    self.is_chassis_moving = True
                    self.get_logger().info(f"底盘新任务: ID:{task_id} Target:{target_pos}")

            else:
                # 抓手逻辑：透传给下位机，不需要上位机 PID
                self.handle_forwarding(device, action, params, task_id)

        except Exception as e:
            self.get_logger().error(f"Brain CMD Error: {e}")

    # ================= 核心逻辑：影子反馈与 PID 闭环 =================
    def shadow_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if data["source"] == "chassis" and self.is_chassis_moving:
                state = data["state"]
                encoders = state.get("motor_encoder", [0, 0, 0, 0])
                
                # 1. 计算平均位置 ( demo 逻辑)
                avg_pos = sum(encoders) / 4.0
                
                # 2. 运行 PID 计算速度
                vx_cmd = self.chassis_pid.compute(avg_pos)
                
                # 3. 终点防抖与到位判定
                # 判定标准：误差 < 50 且 速度指令趋于 0 (停稳)（PID死区）
                if abs(self.chassis_pid.error) < 50 and abs(vx_cmd) < 10:
                    self.stop_chassis()
                    self.notify_protocol_arrival("chassis", self.current_chassis_task_id)
                    self.is_chassis_moving = False
                    self.get_logger().info(f"底盘到位！TaskID: {self.current_chassis_task_id}")
                else:
                    # 发布速度指令给 packer
                    self.dispatch_chassis_speed(vx_cmd)

        except Exception as e:
            self.get_logger().error(f"Shadow Feedback Error: {e}")

    # ================= 工具函数 =================
    def notify_protocol_reset(self, target, task_id):
        """通知解析层：新任务开始，清空到位标志"""
        msg = String()
        msg.data = json.dumps({
            "target": target,
            "update_field": "arrival_done",
            "value": False,
            "task_id": task_id
        })
        self.internal_state_pub.publish(msg)

    def notify_protocol_arrival(self, target, task_id):
        """通知解析层：任务完成，置位到位标志"""
        msg = String()
        msg.data = json.dumps({
            "target": target,
            "update_field": "arrival_done",
            "value": True,
            "task_id": task_id
        })
        self.internal_state_pub.publish(msg)

    def dispatch_chassis_speed(self, vx):
        """包装底盘 0x12 速度指令发送给 Packer"""
        payload = {
            "target": "chassis",
            "sub_id": 0x01, # 车类型
            "cmd_hex": 0x12,
            "params": {"vx": vx, "vy": 0, "vz": 0}
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.packer_pub.publish(msg)

    def stop_chassis(self):
        """发送急停指令"""
        self.dispatch_chassis_speed(0.0)

    def handle_forwarding(self, device, action, params, task_id):
        """抓手类指令透传逻辑 (下位机自带闭环)"""
        # 这里根据 device_tree 查表，将语义转换为 sub_id 和 cmd_hex
        # 示例：handle_left -> lift -> move_to -> 0x01
        # 逻辑同之前的 control_node，但需记得同时通知 Protocol 更新 task_id
        self.notify_protocol_reset(device, task_id)
        # ... 包装并发送 packer_pub ...

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ControlNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()