import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

# =====================================================================
#  BrainNode — 状态机决策节点
#  11 个状态 + DONE，10Hz 轮询 /world_state，发布 /brain_cmd
# =====================================================================


class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')

        # ======================== 状态枚举 ========================
        self.ST_INIT              = 0   # 初始化（设备连接 + 话题健康检查）
        self.ST_WAIT_VISION       = 1   # 等待视觉识别结果
        self.ST_WAIT_START_CMD    = 2   # 等待启动指令
        self.ST_MOVE_TO_GRAB_ZONE = 3   # 底盘→抓豆区 + 抓手预定位（含避障点A）
        self.ST_GRABBING          = 4   # 抓取 + 颜色数据融合 + 理想状态判定
        self.ST_HANDS_TO_AVOID_1  = 5   # 抓手→避障区1
        self.ST_CHASSIS_TO_START  = 6   # 底盘→起始区
        self.ST_HANDS_TO_AVOID_2  = 7   # 抓手→避障区2
        self.ST_CHASSIS_TO_DROP   = 8   # 底盘→放豆区
        self.ST_EXECUTE_TARGET    = 9   # 执行目标移动+放豆（理想/非理想）
        self.ST_CHASSIS_TO_END    = 10  # 底盘→结束区
        self.ST_DONE              = 11  # 任务完成

        self.state = self.ST_INIT
        self.world = None
        self.has_sent_cmd = False          # 状态锁：防止重复下发指令

        # ======================== 方向常量 ========================
        # 由 Brain 决策方向，下位机按指定方向执行
        self.DIR_CW   = 0   # 顺时针（环形轨道）
        self.DIR_CCW  = 1   # 逆时针（环形轨道）
        self.DIR_DOWN = 0   # Z 轴下降
        self.DIR_UP   = 1   # Z 轴上升

        # ======================== 位置参数 ========================
        self._load_positions()

        # ======================== 状态 0：初始化 ========================
        self.init_error_count = 0
        self.init_grace_period = 2.0       # 初始宽容期（秒）
        self.init_enter_time = 0.0          # 进入 INIT 的时间戳
        self.init_checks = {
            "chassis":      {"type": "lower"},
            "handle_left":  {"type": "lower"},
            "handle_mid":   {"type": "lower"},
            "handle_right": {"type": "lower"},
            "vision":       {"type": "upper"},
            "heartbeat":    {"type": "upper"},
        }
        self.init_check_state = {}          # {key: {"error_start": t, "reported": bool}}

        # ======================== 状态 2：启动指令 ========================
        self.start_cmd_received = False

        # ======================== 状态 3：移动到抓豆区 ========================
        self.mid_obstacle_triggered = False
        self.mid_d2_debounce = 0            # D2 触发后等待周期

        # ======================== 状态 4：抓取+融合 ========================
        self.target_seq = None              # 视觉目标序列 [1-5 排列]
        self.is_ideal = False               # 是否理想状态
        self.grab_colors = {}               # {hand: color_id}
        self.target_L = 0                   # 左抓手目标位置
        self.target_M = 0                   # 中抓手目标位置
        self.target_R = 0                   # 右抓手目标位置
        self.grab_seq_step = 0              # 抓取序列步骤 0~10
        self.grab_seq_cmd_sent = False      # 当前步骤指令是否已下发
        self.grab_seq_repeat = 0            # 步骤3-6 循环次数
        self.grab_done = False              # 抓取全流程完成标志
        self.grab_color_snapshot = {}       # 进入状态 4 时的颜色快照

        # ======================== 状态 9：执行目标 ========================
        self._execute_done = False

        # ======================== ROS 接口 ========================
        self.create_subscription(String, '/world_state', self.world_cb, 10)
        self.create_subscription(String, '/task_control', self.start_cmd_cb, 10)
        self.brain_pub = self.create_publisher(String, '/brain_cmd', 10)

        self.create_timer(0.1, self.state_machine_loop)
        self.get_logger().info("Brain Node initialized. Entering INIT state.")

    # =================================================================
    #  回调
    # =================================================================

    def world_cb(self, msg):
        """接收融合层发布的全局影子状态"""
        try:
            self.world = json.loads(msg.data)["data"]
        except Exception as e:
            self.get_logger().error(f"WorldState parse error: {e}")

    def start_cmd_cb(self, msg):
        """接收外部启动指令"""
        try:
            data = json.loads(msg.data)
            if data.get("cmd") == "start":
                self.start_cmd_received = True
                self.get_logger().info("Received START command.")
        except Exception as e:
            self.get_logger().error(f"StartCmd parse error: {e}")

    # =================================================================
    #  指令下发
    # =================================================================

    def dispatch_task(self, device, subsystem, action, params=None):
        """
        下发三段式语义指令到 /brain_cmd。
        device: 目标设备 (chassis, handle_left/mid/right)
        subsystem: 子系统 (base, stepper_x, stepper_z, bldc, servo)
        action: 动作 (move_to, start_grab_cycle, rotate, ...)
        """
        cmd = {
            "device": device,
            "subsystem": subsystem,
            "action": action,
            "params": params if params else {}
        }
        self.brain_pub.publish(String(data=json.dumps(cmd)))
        self.get_logger().info(
            f"Dispatch: {device}.{subsystem} → {action} | params={params}"
        )

    # =================================================================
    #  状态机主循环（10Hz）
    # =================================================================

    def state_machine_loop(self):
        if not self.world:
            return

        if self.state == self.ST_INIT:
            self._handle_init()
        elif self.state == self.ST_WAIT_VISION:
            self._handle_wait_vision()
        elif self.state == self.ST_WAIT_START_CMD:
            self._handle_wait_start()
        elif self.state == self.ST_MOVE_TO_GRAB_ZONE:
            self._handle_move_grab()
        elif self.state == self.ST_GRABBING:
            self._handle_grabbing()
        elif self.state == self.ST_HANDS_TO_AVOID_1:
            self._handle_hands_avoid1()
        elif self.state == self.ST_CHASSIS_TO_START:
            self._handle_chassis_start()
        elif self.state == self.ST_HANDS_TO_AVOID_2:
            self._handle_hands_avoid2()
        elif self.state == self.ST_CHASSIS_TO_DROP:
            self._handle_chassis_drop()
        elif self.state == self.ST_EXECUTE_TARGET:
            self._handle_execute_target()
        elif self.state == self.ST_CHASSIS_TO_END:
            self._handle_chassis_end()
        elif self.state == self.ST_DONE:
            self._handle_done()

    # =================================================================
    #  状态 0：INIT — 初始化
    # =================================================================

    def _handle_init(self):
        """设备在线自检 + 话题健康检查"""

        # 记录进入时间
        if self.init_enter_time == 0.0:
            self.init_enter_time = time.time()
            self.init_check_state = {
                key: {"error_start": 0.0, "reported": False}
                for key in self.init_checks
            }

        now = time.time()

        # 初始宽容期：前 N 秒静默等待，不做检查
        if now - self.init_enter_time < self.init_grace_period:
            return

        all_ok = True

        for key, cfg in self.init_checks.items():
            ok = self._check_item(key)

            if ok:
                # 恢复正常：清除异常记录
                if self.init_check_state[key]["error_start"] != 0.0:
                    self.init_check_state[key] = {"error_start": 0.0, "reported": False}
            else:
                all_ok = False
                state = self.init_check_state[key]
                check_type = cfg["type"]

                if check_type == "lower":
                    # 下位机异常：首次记录时间，等 5 秒再上报
                    if state["error_start"] == 0.0:
                        state["error_start"] = now
                    elif now - state["error_start"] >= 5.0 and not state["reported"]:
                        self.get_logger().error(
                            f"[INIT] 下位机 {key} 连接异常（已等待 {now - state['error_start']:.1f}s）"
                        )
                        state["reported"] = True
                        self.init_error_count += 1
                else:
                    # 上位机异常：立即上报
                    if not state["reported"]:
                        self.get_logger().error(
                            f"[INIT] 上位机 {key} 话题异常"
                        )
                        state["reported"] = True
                        self.init_error_count += 1

        if all_ok:
            self.init_error_count = 0
            self.get_logger().info("[INIT] 全部设备就绪，进入 WAIT_VISION")
            self._transition_to(self.ST_WAIT_VISION)

    def _check_item(self, key):
        """检查单个项目是否正常"""
        try:
            if key == "chassis":
                w = self.world.get("chassis", {})
                return bool(w) and "motor_encoder" in w
            elif key == "handle_left":
                return bool(self.world.get("handles", {}).get("handle_left", {}))
            elif key == "handle_mid":
                return bool(self.world.get("handles", {}).get("handle_mid", {}))
            elif key == "handle_right":
                return bool(self.world.get("handles", {}).get("handle_right", {}))
            elif key == "vision":
                return "vision" in self.world and bool(self.world["vision"])
            elif key == "heartbeat":
                last = self.world.get("last_heartbeat", 0)
                return (time.time() - last) < 2.0
        except Exception:
            return False
        return False

    # =================================================================
    #  状态 1：WAIT_VISION — 等待视觉识别结果
    # =================================================================

    def _handle_wait_vision(self):
        """检查视觉节点是否存活，瞬间通过"""
        if "vision" in self.world and self.world["vision"]:
            self.get_logger().info("[WAIT_VISION] 视觉节点在线，进入 WAIT_START_CMD")
            self._transition_to(self.ST_WAIT_START_CMD)
        else:
            # 异常仅记录，不阻塞，不处理
            self.get_logger().warn("[WAIT_VISION] vision 字段不存在")

    # =================================================================
    #  状态 2：WAIT_START_CMD — 等待启动指令
    # =================================================================

    def _handle_wait_start(self):
        """等待外部 /task_control 下发 {"cmd":"start"}"""
        if self.start_cmd_received:
            self.get_logger().info("[WAIT_START_CMD] 启动指令已收到，进入 MOVE_TO_GRAB_ZONE")
            self._transition_to(self.ST_MOVE_TO_GRAB_ZONE)

    # =================================================================
    #  状态 3：MOVE_TO_GRAB_ZONE — 底盘→抓豆区 + 抓手预定位
    # =================================================================

    def _handle_move_grab(self):
        """
        并行执行:
          A: chassis          → 抓豆区
          B: handle_left      → 6号位（逆时针）
          C: handle_right     → 5号位（顺时针）
          D1: handle_mid      → 3号位（顺时针）
        条件触发:
          D2: 底盘到达避障点A → handle_mid → 2号位（逆时针）
        """
        if not self.has_sent_cmd:
            self.dispatch_task("chassis", "base", "move_to",
                               {"pos": self.POS_GRAB_ZONE})
            self._move_hand_x("handle_left",  self.POS_6, self.DIR_CCW)
            self._move_hand_x("handle_right", self.POS_5, self.DIR_CW)
            self._move_hand_x("handle_mid",   self.POS_3, self.DIR_CW)
            self.has_sent_cmd = True
            self.mid_obstacle_triggered = False
            self.mid_d2_debounce = 0
            self.get_logger().info("[MOVE_TO_GRAB] 4路并行指令已下发")

        # --- 避障点 A 触发检查 ---
        if not self.mid_obstacle_triggered:
            avg_pos = self._get_chassis_avg_pos()
            if avg_pos is not None and avg_pos >= self.POS_OBSTACLE_A:
                self._move_hand_x("handle_mid", self.POS_2, self.DIR_CCW)
                self.mid_obstacle_triggered = True
                self.mid_d2_debounce = 0
                self.get_logger().info(
                    f"[MOVE_TO_GRAB] 底盘到达避障点A(pos={avg_pos})，触发D2"
                )

        # --- D2 触发后消抖：等 3 个周期让 track_arrived 被重置 ---
        if self.mid_obstacle_triggered and self.mid_d2_debounce < 3:
            self.mid_d2_debounce += 1
            return

        # --- 全部到位检查 ---
        chassis_ok = self.world.get("chassis", {}).get("arrival_done", False)
        left_ok = self.world.get("handles", {}).get("handle_left", {}).get("track_arrived", False)
        right_ok = self.world.get("handles", {}).get("handle_right", {}).get("track_arrived", False)
        mid_ok = (self.world.get("handles", {}).get("handle_mid", {}).get("track_arrived", False)
                  and self.mid_obstacle_triggered)

        if chassis_ok and left_ok and right_ok and mid_ok:
            self.get_logger().info("[MOVE_TO_GRAB] 全部到位，进入 GRABBING")
            self._transition_to(self.ST_GRABBING)

    # =================================================================
    #  状态 4：GRABBING — 抓取 + 颜色数据融合 + 理想状态判定
    # =================================================================

    def _handle_grabbing(self):
        """
        阶段一（抓取动作序列）与 阶段二（等待颜色更新）并行。
        grab_bean() 每周期推进序列步骤。
        颜色全部更新 + 抓取完成 → 阶段三（数据融合）→ 进入下一状态。
        """
        handles = ["handle_left", "handle_mid", "handle_right"]

        # --- 初始化 ---
        if not self.has_sent_cmd:
            self.grab_seq_step = 0
            self.grab_seq_cmd_sent = False
            self.grab_seq_repeat = 0
            self.grab_done = False
            # 同时开始阶段二：记录当前颜色作为快照
            self.grab_color_snapshot = {}
            for h in handles:
                self.grab_color_snapshot[h] = (
                    self.world.get("handles", {}).get(h, {}).get("color_id", 0)
                )
            self.has_sent_cmd = True
            self.get_logger().info(
                f"[GRABBING] 进入抓取状态，颜色快照: {self.grab_color_snapshot}"
            )

        # --- 阶段一：每周期推进抓取序列 ---
        if not self.grab_done:
            self.grab_bean()

        # --- 阶段二：等待颜色数据更新 ---
        all_updated = True
        for h in handles:
            current = self.world.get("handles", {}).get(h, {}).get("color_id", 0)
            if current == 0 or current == self.grab_color_snapshot.get(h, 0):
                all_updated = False
                break

        # --- 阶段三：两者都完成 → 数据融合 ---
        if self.grab_done and all_updated:
            grabbed = {}
            for h in handles:
                grabbed[h] = self.world.get("handles", {}).get(h, {}).get("color_id", 0)
            self._data_fusion(grabbed)
            self.get_logger().info(
                f"[GRABBING] 数据融合完成: colors={grabbed}, "
                f"targets=({self.target_L},{self.target_M},{self.target_R}), "
                f"is_ideal={self.is_ideal}"
            )
            self._transition_to(self.ST_HANDS_TO_AVOID_1)

    def _data_fusion(self, grabbed_colors):
        """
        颜色 → 箱号 → 轨道位置 → 判定理想状态。
        color_id: 1=黄豆→Box1, 2=绿豆→Box2, 3=白芸豆→Box3
        放豆区 5 个箱在轨道上的位置（顺时针）: [7, 1, 2, 3, 4]
        """
        self.grab_colors = grabbed_colors

        # 颜色→箱号（同号直接映射）
        color_to_box = {1: 1, 2: 2, 3: 3}
        # 放豆区箱子顺时针对应轨道位置
        # target_seq[0]→pos7, [1]→pos1, [2]→pos2, [3]→pos3, [4]→pos4
        DROP_TRACK_POS = [7, 1, 2, 3, 4]

        box_targets = {}
        for hand in ["handle_left", "handle_mid", "handle_right"]:
            color = grabbed_colors.get(hand, 0)
            box_id = color_to_box.get(color, 0)
            if box_id > 0 and self.target_seq and box_id in self.target_seq:
                idx = self.target_seq.index(box_id)
                box_targets[hand] = DROP_TRACK_POS[idx]
            else:
                box_targets[hand] = 0

        self.target_L = box_targets.get("handle_left", 0)
        self.target_M = box_targets.get("handle_mid", 0)
        self.target_R = box_targets.get("handle_right", 0)

        self.is_ideal = self._check_ideal()

    def _check_ideal(self):
        """
        颜色传感器 + 视觉融合，判定理想/非理想状态。
        理想状态：三个抓手可以同时放豆。
        非理想状态：需要分两次放豆。
        """
        targets = [self.target_L, self.target_M, self.target_R]
        if 0 in targets:
            return False
        if len(set(targets)) != 3:
            return False
        return True

    # =================================================================
    #  状态 5：HANDS_TO_AVOID_1 — 抓手→避障区1
    # =================================================================

    def _handle_hands_avoid1(self):
        """左逆→5, 右逆→4, 中逆→1"""
        if not self.has_sent_cmd:
            self._move_hand_x("handle_left",  self.POS_5, self.DIR_CCW)
            self._move_hand_x("handle_right", self.POS_4, self.DIR_CCW)
            self._move_hand_x("handle_mid",   self.POS_1, self.DIR_CCW)
            self.has_sent_cmd = True
            self.get_logger().info("[AVOID_1] 3路并行指令已下发")

        if self._all_hands_arrived():
            self.get_logger().info("[AVOID_1] 全部到位，进入 CHASSIS_TO_START")
            self._transition_to(self.ST_CHASSIS_TO_START)

    # =================================================================
    #  状态 6：CHASSIS_TO_START — 底盘→起始区
    # =================================================================

    def _handle_chassis_start(self):
        if not self.has_sent_cmd:
            self.dispatch_task("chassis", "base", "move_to",
                               {"pos": self.POS_START_ZONE})
            self.has_sent_cmd = True
            self.get_logger().info("[CHASSIS_TO_START] 指令已下发")

        if self.world.get("chassis", {}).get("arrival_done", False):
            self.get_logger().info("[CHASSIS_TO_START] 到位，进入 AVOID_2")
            self._transition_to(self.ST_HANDS_TO_AVOID_2)

    # =================================================================
    #  状态 7：HANDS_TO_AVOID_2 — 抓手→避障区2
    # =================================================================

    def _handle_hands_avoid2(self):
        """左顺→7, 右顺→6, 中顺→3"""
        if not self.has_sent_cmd:
            self._move_hand_x("handle_left",  self.POS_7, self.DIR_CW)
            self._move_hand_x("handle_right", self.POS_6, self.DIR_CW)
            self._move_hand_x("handle_mid",   self.POS_3, self.DIR_CW)
            self.has_sent_cmd = True
            self.get_logger().info("[AVOID_2] 3路并行指令已下发")

        if self._all_hands_arrived():
            self.get_logger().info("[AVOID_2] 全部到位，进入 CHASSIS_TO_DROP")
            self._transition_to(self.ST_CHASSIS_TO_DROP)

    # =================================================================
    #  状态 8：CHASSIS_TO_DROP — 底盘→放豆区
    # =================================================================

    def _handle_chassis_drop(self):
        if not self.has_sent_cmd:
            self.dispatch_task("chassis", "base", "move_to",
                               {"pos": self.POS_DROP_ZONE})
            self.has_sent_cmd = True
            self.get_logger().info("[CHASSIS_TO_DROP] 指令已下发")

        if self.world.get("chassis", {}).get("arrival_done", False):
            self.get_logger().info("[CHASSIS_TO_DROP] 到位，进入 EXECUTE_TARGET")
            self._transition_to(self.ST_EXECUTE_TARGET)

    # =================================================================
    #  状态 9：EXECUTE_TARGET — 执行目标移动+放豆
    # =================================================================

    def _handle_execute_target(self):
        """
        调用 execute_drop() 处理抓手移动+放豆（理想/非理想）。
        状态机不关心内部细节，只等待 _execute_done 标志。
        """
        if not self.has_sent_cmd:
            self._execute_done = False
            self.get_logger().info("[EXECUTE_TARGET] 进入放豆执行状态")
            self.has_sent_cmd = True

        if not self._execute_done:
            self.execute_drop()

        if self._execute_done:
            self.get_logger().info("[EXECUTE_TARGET] 执行完毕，进入 CHASSIS_TO_END")
            self._transition_to(self.ST_CHASSIS_TO_END)

    def execute_drop(self):
        """
        抓手移动到放豆位置并放豆。
        内部根据 self.is_ideal 分支处理理想/非理想路径。
        由状态机每周期调用，执行完毕设置 self._execute_done = True。
        TODO: 后续补充完整逻辑（移动+舵机放豆+调整+最后放豆）。
        """
        if self.is_ideal:
            self.get_logger().info("[DROP] 理想状态执行 - 占位")
        else:
            self.get_logger().info("[DROP] 非理想状态执行 - 占位")
        self._execute_done = True

    # =================================================================
    #  状态 10：CHASSIS_TO_END — 底盘→结束区
    # =================================================================

    def _handle_chassis_end(self):
        if not self.has_sent_cmd:
            self.dispatch_task("chassis", "base", "move_to",
                               {"pos": self.POS_END_ZONE})
            self.has_sent_cmd = True
            self.get_logger().info("[CHASSIS_TO_END] 指令已下发")

        if self.world.get("chassis", {}).get("arrival_done", False):
            self.get_logger().info("[CHASSIS_TO_END] 到位，任务完成")
            self._transition_to(self.ST_DONE)

    # =================================================================
    #  状态 11：DONE — 任务完成
    # =================================================================

    def _handle_done(self):
        self.get_logger().info("=" * 50)
        self.get_logger().info("  全部任务完成！Brain Node 即将关闭。")
        self.get_logger().info("=" * 50)
        rclpy.shutdown()

    # =================================================================
    #  工具方法
    # =================================================================

    def _transition_to(self, new_state):
        """状态切换，重置状态锁"""
        self.state = new_state
        self.has_sent_cmd = False

    def _get_chassis_avg_pos(self):
        """获取底盘平均编码器位置"""
        try:
            encoders = self.world.get("chassis", {}).get("motor_encoder", [])
            if encoders and len(encoders) == 4:
                return sum(encoders) / 4.0
        except Exception:
            pass
        return None

    def grab_bean(self):
        """
        抓取动作序列，每周期由 _handle_grabbing 调用，推进一个步骤。
        步骤: 1下→2开→3顺→4降→5逆→6降→[循环5次]→8标→9关→10升
        """
        h = ["handle_left", "handle_mid", "handle_right"]

        # --- 步骤0: 初始化 ---
        if self.grab_seq_step == 0:
            self.grab_seq_step = 1
            self.grab_seq_cmd_sent = False
            self.grab_seq_repeat = 0

        # --- 步骤1: Z轴下降到 GRAB_Z_A（绝对位置） ---
        elif self.grab_seq_step == 1:
            if not self.grab_seq_cmd_sent:
                for hand in h:
                    self.dispatch_task(hand, "stepper_z", "move_to",
                                       {"pos": self.GRAB_Z_A, "dir": self.DIR_DOWN})
                self.grab_seq_cmd_sent = True
                self.get_logger().info("[GRAB:1/10] Z轴下降至 GRAB_Z_A")
            if self._all_hands_arrived():
                self._next_grab_step()

        # --- 步骤2: 开启无刷电机 占空比100% ---
        elif self.grab_seq_step == 2:
            for hand in h:
                self.dispatch_task(hand, "bldc", "start", {"duty": 100})
            self.get_logger().info("[GRAB:2/10] 无刷电机启动 100%")
            self._next_grab_step()

        # --- 步骤3: X轴顺时针 → GRAB_X_CW（绝对位置） ---
        elif self.grab_seq_step == 3:
            if not self.grab_seq_cmd_sent:
                for hand in h:
                    self._move_hand_x(hand, self.GRAB_X_CW, self.DIR_CW)
                self.grab_seq_cmd_sent = True
                self.get_logger().info(
                    f"[GRAB:3/10] X轴顺时针 → GRAB_X_CW (第{self.grab_seq_repeat + 1}次)"
                )
            if self._all_hands_arrived():
                self._next_grab_step()

        # --- 步骤4: Z轴相对下降 GRAB_Z_DELTA ---
        elif self.grab_seq_step == 4:
            if not self.grab_seq_cmd_sent:
                for hand in h:
                    self.dispatch_task(hand, "stepper_z", "move_relative",
                                       {"pos": self.GRAB_Z_DELTA, "dir": self.DIR_DOWN})
                self.grab_seq_cmd_sent = True
                self.get_logger().info(
                    f"[GRAB:4/10] Z轴下降 {self.GRAB_Z_DELTA} (第{self.grab_seq_repeat + 1}次)"
                )
            if self._all_hands_arrived():
                self._next_grab_step()

        # --- 步骤5: X轴逆时针 → GRAB_X_CCW（绝对位置） ---
        elif self.grab_seq_step == 5:
            if not self.grab_seq_cmd_sent:
                for hand in h:
                    self._move_hand_x(hand, self.GRAB_X_CCW, self.DIR_CCW)
                self.grab_seq_cmd_sent = True
                self.get_logger().info(
                    f"[GRAB:5/10] X轴逆时针 → GRAB_X_CCW (第{self.grab_seq_repeat + 1}次)"
                )
            if self._all_hands_arrived():
                self._next_grab_step()

        # --- 步骤6: Z轴相对下降 GRAB_Z_DELTA ---
        elif self.grab_seq_step == 6:
            if not self.grab_seq_cmd_sent:
                for hand in h:
                    self.dispatch_task(hand, "stepper_z", "move_relative",
                                       {"pos": self.GRAB_Z_DELTA, "dir": self.DIR_DOWN})
                self.grab_seq_cmd_sent = True
                self.get_logger().info(
                    f"[GRAB:6/10] Z轴下降 {self.GRAB_Z_DELTA} (第{self.grab_seq_repeat + 1}次)"
                )
            if self._all_hands_arrived():
                self.grab_seq_repeat += 1
                if self.grab_seq_repeat < 5:
                    # 回到步骤3 继续循环
                    self.grab_seq_step = 3
                    self.grab_seq_cmd_sent = False
                else:
                    self.grab_seq_step = 8    # 跳过步骤7（重复逻辑），直达步骤8
                    self.grab_seq_cmd_sent = False

        # --- 步骤8: 设置抓取完成标志位 ---
        elif self.grab_seq_step == 8:
            self.get_logger().info("[GRAB:8/10] 抓取完成标志位已设置")
            self._next_grab_step()

        # --- 步骤9: 关闭无刷电机 ---
        elif self.grab_seq_step == 9:
            for hand in h:
                self.dispatch_task(hand, "bldc", "stop", {})
            self.get_logger().info("[GRAB:9/10] 无刷电机关闭")
            self._next_grab_step()

        # --- 步骤10: Z轴上升到 GRAB_Z_F（绝对位置） ---
        elif self.grab_seq_step == 10:
            if not self.grab_seq_cmd_sent:
                for hand in h:
                    self.dispatch_task(hand, "stepper_z", "move_to",
                                       {"pos": self.GRAB_Z_F, "dir": self.DIR_UP})
                self.grab_seq_cmd_sent = True
                self.get_logger().info("[GRAB:10/10] Z轴上升至 GRAB_Z_F")
            if self._all_hands_arrived():
                self.grab_done = True
                self.get_logger().info("[GRAB] 抓取全流程完成")

    def _next_grab_step(self):
        """抓取序列推进到下一步骤"""
        self.grab_seq_step += 1
        self.grab_seq_cmd_sent = False

    def _hand_encoder(self, hand, axis="x"):
        """读取单个抓手当前编码器值"""
        addr = 0x01 if axis == "x" else 0x02
        motor = (self.world.get("handles", {})
                 .get(hand, {}).get("stepmotor", {}).get(addr, {}))
        return motor.get("current_pos") if motor else None

    def _ring_displacement(self, current, target, direction):
        """
        环形轨道相对脉冲计算。
        current:  当前编码器值
        target:   position.yaml 中的物理位置值
        direction: DIR_CW / DIR_CCW
        返回相对脉冲数。
        """
        ring = self.RING_MAX
        if direction == self.DIR_CW:
            if target >= current:
                return target - current
            else:
                return ring - current + target
        else:
            if target <= current:
                return current - target
            else:
                return current + ring - target

    def _move_hand_x(self, hand, target, direction):
        """单抓手 X 轴环形移动：读编码器 → 算位移 → 下 move_relative"""
        cur = self._hand_encoder(hand, "x")
        if cur is None:
            self.get_logger().error(f"[MOVE] {hand} encoder read failed")
            return
        dist = self._ring_displacement(cur, target, direction)
        self.dispatch_task(hand, "stepper_x", "move_relative",
                           {"pos": dist, "dir": direction})

    def _all_hands_arrived(self):
        """检查三个抓手是否全部到位（通过 /world_state 中的 track_arrived 字段）"""
        handles = self.world.get("handles", {})
        return all(
            handles.get(h, {}).get("track_arrived", False)
            for h in ["handle_left", "handle_mid", "handle_right"]
        )

    def _load_positions(self):
        """
        从 config/position.yaml 加载位置参数，失败则使用默认值。
        """
        import yaml, os
        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_path = os.path.join(pkg_dir, 'config', 'position.yaml')

        try:
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f)

            # --- 轨道位置 ---
            r = cfg["rail"]
            self.RING_MAX = r["ring_max"]
            self.POS_1 = r["pos_1"]
            self.POS_2 = r["pos_2"]
            self.POS_3 = r["pos_3"]
            self.POS_4 = r["pos_4"]
            self.POS_5 = r["pos_5"]
            self.POS_6 = r["pos_6"]
            self.POS_7 = r["pos_7"]

            # --- 底盘目标位置 ---
            c = cfg["chassis"]
            self.POS_START_ZONE = c["start_zone"]
            self.POS_GRAB_ZONE = c["grab_zone"]
            self.POS_DROP_ZONE = c["drop_zone"]
            self.POS_END_ZONE = c["end_zone"]
            self.POS_OBSTACLE_A = c["obstacle_a"]

            # --- 抓取动作参数 ---
            g = cfg["grab"]
            self.GRAB_Z_A = g["z_down_a"]
            self.GRAB_X_CW = g["x_cw"]
            self.GRAB_X_CCW = g["x_ccw"]
            self.GRAB_Z_DELTA = g["z_delta"]
            self.GRAB_Z_F = g["z_up"]

            self.get_logger().info(f"Loaded position params from {config_path}")

        except Exception as e:
            self.get_logger().error(
                f"Failed to load {config_path}: {e}. Using defaults."
            )
            self._set_default_positions()

    def _set_default_positions(self):
        """硬编码默认值（YAML 加载失败时的兜底）"""
        self.RING_MAX = 178000
        self.POS_1 = 0
        self.POS_2 = 150
        self.POS_3 = 300
        self.POS_4 = 450
        self.POS_5 = 600
        self.POS_6 = 750
        self.POS_7 = 900

        self.POS_START_ZONE = 0
        self.POS_GRAB_ZONE = 1500
        self.POS_DROP_ZONE = 3000
        self.POS_END_ZONE = 0
        self.POS_OBSTACLE_A = 800

        self.GRAB_Z_A = 500
        self.GRAB_X_CW = 800
        self.GRAB_X_CCW = 300
        self.GRAB_Z_DELTA = 50
        self.GRAB_Z_F = 200

    # =================================================================
    #  便于外部查看的状态名称映射
    # =================================================================

    @property
    def state_name(self):
        names = {
            0:  "INIT",
            1:  "WAIT_VISION",
            2:  "WAIT_START_CMD",
            3:  "MOVE_TO_GRAB_ZONE",
            4:  "GRABBING",
            5:  "HANDS_TO_AVOID_1",
            6:  "CHASSIS_TO_START",
            7:  "HANDS_TO_AVOID_2",
            8:  "CHASSIS_TO_DROP",
            9:  "EXECUTE_TARGET",
            10: "CHASSIS_TO_END",
            11: "DONE",
        }
        return names.get(self.state, f"UNKNOWN({self.state})")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(BrainNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
