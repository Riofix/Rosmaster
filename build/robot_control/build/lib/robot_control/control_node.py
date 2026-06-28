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
                "stepper_x": {"motor_addr": 0x01, "type": "handle_stepper"},
                "stepper_z": {"motor_addr": 0x02, "type": "handle_stepper"},
                "servo": {"channel": 0x00, "type": "handle_servo"},
                "bldc": {"type": "handle_bldc"}
            },
            "handle_mid": {
                "stepper_x": {"motor_addr": 0x01, "type": "handle_stepper"},
                "stepper_z": {"motor_addr": 0x02, "type": "handle_stepper"},
                "servo": {"channel": 0x00, "type": "handle_servo"},
                "bldc": {"type": "handle_bldc"}
            },
            "handle_right": {
                "stepper_x": {"motor_addr": 0x01, "type": "handle_stepper"},
                "stepper_z": {"motor_addr": 0x02, "type": "handle_stepper"},
                "servo": {"channel": 0x00, "type": "handle_servo"},
                "bldc": {"type": "handle_bldc"}
            },
        }

        # 指令码定义
        self.cmd_map = {
            "mecanum_base": {"set_speed": 0x12},
            "handle_stepper": {
                "enable": 0x60,
                "disable": 0x60,
                "velocity": 0x61,
                "move_relative": 0x62,
                "move_absolute": 0x62,
                "stop": 0x63
            },
            "handle_servo": {"move_to": 0x6C, "set_angle": 0x6C},
            "handle_bldc": {"start": 0x6D, "stop": 0x6E}
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
            if task.get("subsystem") is not None:
                params["subsystem"] = task.get("subsystem")

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
        """通知解析层：新任务开始，清空到位标志 + 写入 task_id"""
        if target == "chassis":
            field = "arrival_done"
        else:
            field = "track_arrived"
        msg = String()
        msg.data = json.dumps({
            "target": target,
            "update_field": field,
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
        """Forward handle commands without touching chassis control."""
        subsystem = params.get("subsystem") or params.get("sub_system") or params.get("part")
        handle_config = self.device_tree.get(device)
        if handle_config is None:
            self.get_logger().warn(f"Unknown handle device: {device}")
            return

        sub_config = handle_config.get(subsystem)
        if sub_config is None:
            self.get_logger().warn(f"Unknown handle subsystem: {device}.{subsystem}")
            return

        cmd_hex = self.cmd_map.get(sub_config["type"], {}).get(action)
        if cmd_hex is None:
            self.get_logger().warn(f"Unsupported handle action: {device}.{subsystem}.{action}")
            return

        self.notify_protocol_reset(device, task_id)

        control_params = dict(params)
        control_params.pop("subsystem", None)
        control_params.pop("sub_system", None)
        control_params.pop("part", None)

        if sub_config["type"] == "handle_stepper":
            control_params["motor_addr"] = sub_config["motor_addr"]
            control_params["relative"] = action == "move_relative"
            if action == "enable":
                control_params["enable"] = True
            elif action == "disable":
                control_params["enable"] = False
        elif sub_config["type"] == "handle_servo":
            control_params["channel"] = control_params.get("channel", sub_config["channel"])

        payload = {
            "target": device,
            "cmd_hex": cmd_hex,
            "params": control_params
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.packer_pub.publish(msg)
        self.get_logger().info(f"Forwarded handle command: {device}.{subsystem}.{action}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ControlNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
