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
        self.ST_RESET             = 12  # 复位: 回起始位 + 编码器清零

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

        # 底盘测试指令 → 脉冲值映射
        self._goto_positions = {
            "goto_start_zone":  self.POS_START_ZONE,
            "goto_obstacle_b":  self.POS_OBSTACLE_B,
            "goto_place_zone":  self.POS_DROP_ZONE,
            "goto_obstacle_a":  self.POS_OBSTACLE_A,
            "goto_grab_zone":   self.POS_GRAB_ZONE,
        }

        # ======================== 环形轨道拓扑 ========================
        # 顺时针: 1→7→2→3→4→8→6→5→(回1)
        self._cw_next = {1:7, 7:2, 2:3, 3:4, 4:8, 8:6, 6:5, 5:1}
        self._ccw_next = {7:1, 2:7, 3:2, 4:3, 8:4, 6:8, 5:6, 1:5}

        # 环形轨道边代价（无向）
        self._edge_cost = {
            1: {7: 0.1, 5: 0.1},
            7: {1: 0.1, 2: 0.01},
            2: {7: 0.01, 3: 0.1},
            3: {2: 0.1, 4: 0.1},
            4: {3: 0.1, 8: 0.2},
            8: {4: 0.2, 6: 0.25},
            6: {8: 0.25, 5: 0.2},
            5: {6: 0.2, 1: 0.1},
        }

        # ======================== 状态 0：初始化 (plan_readme 4.2) ========================
        self.init_step = 0              # 步骤 0~10
        self.init_step_cmd_sent = False
        self._init_stall_state = {}     # {hand: 0=等堵转, 1=停, 2=清零, 3=done}

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

        # ======================== 状态 9：放豆执行计划 ========================
        self._execute_done = False
        self.drop_plan = []           # [batch, ...] 每批 {hand: (pulse_target, dir)}
        self._drop_batch = 0          # 当前批次索引
        self._drop_step = 0           # 0=发移动 1=等到位 2=舵机放豆
        self._drop_step_timer = 0     # 舵机步骤消抖计数器

        # ======================== 全局指令计数器 ========================
        self._task_counter = 0        # 自增 task_id

        # ======================== 调试控制 ========================
        self.declare_parameter('debug_mode', True)  # launch 可控
        self.step_mode = self.get_parameter('debug_mode').value
        self.step_paused = True         # 单步暂停标志
        self.estop_locked = False       # 急停锁定, 只有 reset 能解除
        self._step_prev_state = self.state  # 记录上一步状态, 检测状态变化

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
        """接收外部指令"""
        try:
            data = json.loads(msg.data)
            cmd = data.get("cmd", "")

            # ── 流程控制 ──
            if cmd == "start":
                self.start_cmd_received = True
                self.get_logger().info("[CMD] 全流程启动")

            # ── 底盘测试移动 ──
            elif cmd in self._goto_positions:
                target = self._goto_positions[cmd]
                self.dispatch_task("chassis", "base", "move_to", {"pos": target})
                self.get_logger().info(f"[CMD] GOTO: {cmd} → pos={target}")

            # ── 单步/自动切换 ──
            elif cmd == "step_mode":
                if self.estop_locked:
                    self.get_logger().warn("[CMD] step_mode 无效: 已急停锁定, 请先 reset")
                    return
                self.step_mode = True
                self.step_paused = True
                self.get_logger().info(f"[CMD] 进入单步模式 → 当前 {self.state_name}")

            elif cmd == "step_next":
                if self.estop_locked:
                    self.get_logger().warn("[CMD] step_next 无效: 已急停锁定, 请先 reset")
                    return
                if not self.step_mode:
                    self.get_logger().warn("[CMD] step_next 无效: 非单步模式, 请先 step_mode")
                    return
                self.step_paused = False
                self.get_logger().info(
                    f"[CMD] 单步推进 → 从 {self.state_name} 开始"
                )

            # ── 急停 (锁死, 只有 reset 可解除) ──
            elif cmd == "estop":
                self._do_emergency_stop()
                self.estop_locked = True
                self.step_paused = True
                self.get_logger().info("[CMD] 🛑 紧急停止 (已锁定, 请 reset)")

            # ── 复位 (急停 + 解锁 + 进 ST_RESET) ──
            elif cmd == "reset":
                self.estop_locked = True   # 复位期间锁定, estop 可打断
                self.step_paused = False   # 解除单步暂停让 ST_RESET 跑起来
                self._do_emergency_stop()
                self._transition_to(self.ST_RESET)
                self.get_logger().info("[CMD] 复位 → ST_RESET")

        except Exception as e:
            self.get_logger().error(f"StartCmd parse error: {e}")

    def _do_emergency_stop(self):
        """急停: 底盘刹车 + 三抓手 0x7D"""
        self.dispatch_task("chassis", "base", "stop", {})
        for h in ["handle_left", "handle_mid", "handle_right"]:
            self.dispatch_task(h, "stepper_x", "emergency_stop", {})

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
        self._task_counter += 1
        cmd = {
            "device": device,
            "subsystem": subsystem,
            "action": action,
            "task_id": self._task_counter,
            "params": params if params else {}
        }
        self.brain_pub.publish(String(data=json.dumps(cmd)))
        self.get_logger().info(
            f"Dispatch: {device}.{subsystem} → {action} "
            f"| task_id={self._task_counter} | params={params}"
        )

    # =================================================================
    #  状态机主循环（10Hz）
    # =================================================================

    def state_machine_loop(self):
        if not self.world:
            return

        # ── 急停锁定: 只有 ST_RESET 可以穿透, 其余冻结 ──
        if self.estop_locked and self.state != self.ST_RESET:
            return

        # ── 单步模式: 暂停等待 step_next ──
        if self.step_mode and self.step_paused:
            return

        # ── 记录单步开始时的状态 ──
        if self.step_mode:
            self._step_prev_state = self.state

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
        elif self.state == self.ST_RESET:
            self._handle_reset()

        # ── 单步模式: 状态发生变化时自动暂停 ──
        if self.step_mode and self.state != self._step_prev_state:
            self.step_paused = True
            self.get_logger().info(
                f"[单步] {self.state_name}, 已暂停 (发 step_next 继续)"
            )

    # =================================================================
    #  状态 0：INIT — 初始化 (plan_readme 4.2, 步骤 0~10)
    # =================================================================

    def _handle_init(self):
        """逐步执行初始化流程, 每周期推进一次"""
        handles = ["handle_left", "handle_mid", "handle_right"]

        # ── 步骤 0: 设备连接检查 ──
        if self.init_step == 0:
            try:
                chassis_ok = bool(self.world.get("chassis", {}).get("motor_encoder"))
                h_ok = all(
                    bool(self.world.get("handles", {}).get(h, {}))
                    for h in handles
                )
                if chassis_ok and h_ok:
                    self.get_logger().info("[INIT 0/10] 设备全部在线")
                    self.init_step = 0.5
                    self.init_step_cmd_sent = False
            except Exception:
                pass

        # ── 步骤 0.5: 开启自动上报 ──
        elif self.init_step == 0.5:
            if not self.init_step_cmd_sent:
                for h in handles:
                    self.dispatch_task(h, "stream", "mpu_enable", {"enable": 1})
                    self.dispatch_task(h, "stream", "step_enable", {"enable": 1})
                    self.dispatch_task(h, "stream", "pwm_enable", {"enable": 1})
                self.init_step_cmd_sent = True
                self.get_logger().info("[INIT 0.5/10] 自动上报已开启")
            self.init_step = 0.6
            self.init_step_cmd_sent = False

        # ── 步骤 0.6: OLED 智能模式 ──
        elif self.init_step == 0.6:
            if not self.init_step_cmd_sent:
                for h in handles:
                    self.dispatch_task(h, "stream", "oled_mode", {"mode": 9})
                self.init_step_cmd_sent = True
                self.get_logger().info("[INIT 0.6/10] OLED → 智能模式")
            self.init_step = 1
            self.init_step_cmd_sent = False

        # ── 步骤 1~3: 校准原点 (左=1, 中=2, 右=3) ──
        elif self.init_step in (1, 2, 3):
            mapping = {1: ("handle_left",  1),
                       2: ("handle_mid",   2),
                       3: ("handle_right", 3)}
            if self.init_step in mapping:
                h, pos_id = mapping[self.init_step]
                if not self.init_step_cmd_sent:
                    self.dispatch_task(h, "stepper_x", "set_origin",
                                       {"pos_id": pos_id})
                    self.init_step_cmd_sent = True
                    self.get_logger().info(f"[INIT {self.init_step}/10] {h} 校准原点 pos={pos_id}")
                # set_origin 是即时指令, 等下个周期推进
                self.init_step = self.init_step + 1
                self.init_step_cmd_sent = False

        # ── 步骤 4~7: 电机2堵转回零 (每抓手独立) ──
        # state: 0=等堵转保护(0x08) → 1=清零 → 2=解保护 → 3=等保护解除 → 4=完成
        elif self.init_step == 4:
            if not self.init_step_cmd_sent:
                for h in handles:
                    self.dispatch_task(h, "stepper_z", "velocity",
                                       {"dir": 0, "speed": 500, "acc": 100})
                self._init_stall_state = {h: 0 for h in handles}
                self.init_step_cmd_sent = True
                self.get_logger().info("[INIT 4/10] 电机2上升, 各抓手独立检测...")

            for h in handles:
                state = self._init_stall_state.get(h, 0)
                if state >= 4:
                    continue

                flag = (self.world.get("handles", {})
                        .get(h, {}).get("stepmotor", {})
                        .get("2", {}).get("flag", 0))

                if state == 0:
                    # 等待堵转保护触发 (flag bit4, 0x08)
                    if flag & 0x08:
                        self.dispatch_task(h, "stepper_z", "stop", {})
                        self._init_stall_state[h] = 1
                        self.get_logger().info(f"[INIT 4/10] {h} 堵转保护触发 → 停止")

                elif state == 1:
                    self.dispatch_task(h, "stepper_z", "reset_encoder", {})
                    self._init_stall_state[h] = 2
                    self.get_logger().info(f"[INIT 4/10] {h} 编码器清零")

                elif state == 2:
                    self.dispatch_task(h, "stepper_z", "reset_clog", {})
                    self._init_stall_state[h] = 3
                    self.get_logger().info(f"[INIT 4/10] {h} 发0x6B解保护")

                elif state == 3:
                    # 确认堵转保护已解除 (flag bit3 清零)
                    if not (flag & 0x08):
                        self._init_stall_state[h] = 4
                        self.get_logger().info(f"[INIT 4/10] {h} 堵转保护已解除, 完成")
                    else:
                        self.dispatch_task(h, "stepper_z", "reset_clog", {})
                        self.get_logger().info(f"[INIT 4/10] {h} 堵转保护未解除, 重发0x6B")

            # 全部完成 → 步骤 8
            if all(s >= 4 for s in self._init_stall_state.values()):
                self.get_logger().info("[INIT 4/10] 三抓手堵转回零全部完成")
                self.init_step = 8
                self.init_step_cmd_sent = False

        # ── 步骤 8: 舵机闭合 90° ──
        elif self.init_step == 8:
            if not self.init_step_cmd_sent:
                for h in handles:
                    self.dispatch_task(h, "servo", "move_to", {"angle": 90})
                self.init_step_cmd_sent = True
                self.get_logger().info("[INIT 8/10] 舵机闭合 90°")
            self.init_step = 9
            self.init_step_cmd_sent = False

        # ── 步骤 9: 获取视觉序列 ──
        elif self.init_step == 9:
            seq = self.world.get("vision", {}).get("sequence", [])
            if seq and len(seq) == 5:
                self.target_seq = seq
                self.get_logger().info(f"[INIT 9/10] 视觉序列: {seq}")
                self.init_step = 10
                self.init_step_cmd_sent = False

        # ── 步骤 10: 完成 → WAIT_START_CMD ──
        elif self.init_step == 10:
            self.get_logger().info("[INIT 10/10] 初始化完成 → WAIT_START_CMD")
            self._transition_to(self.ST_WAIT_START_CMD)

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
            self._move_hand_to("handle_left",  6, self.DIR_CCW)
            self._move_hand_to("handle_right", 8, self.DIR_CW)
            self._move_hand_to("handle_mid",   3, self.DIR_CW)
            self.has_sent_cmd = True
            self.mid_obstacle_triggered = False
            self.mid_d2_debounce = 0
            self.get_logger().info("[MOVE_TO_GRAB] 4路并行指令已下发")

        # --- 避障点 A 触发检查 ---
        chassis_arrived = self.world.get("chassis", {}).get("arrival_done", False)
        if not self.mid_obstacle_triggered:
            avg_pos = self._get_chassis_avg_pos()
            # S方向(负), encoder ≤ -40095 或底盘已到位 都算越过
            if (avg_pos is not None and avg_pos <= self.POS_OBSTACLE_A) or chassis_arrived:
                self._move_hand_to("handle_mid", 7, self.DIR_CCW)
                self.mid_obstacle_triggered = True
                self.mid_d2_debounce = 0
                self.get_logger().info(
                    f"[MOVE_TO_GRAB] 触发D2 (pos={avg_pos}, arrived={chassis_arrived})"
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
        1. 发 grab_start(0x79) 三抓手并行
        2. 等待 action_done(0x7C)
        3. 读 color_id → 数据融合 → 下一状态
        """
        handles = ["handle_left", "handle_mid", "handle_right"]

        # ── 阶段 0: 发抓取指令 ──
        if not self.has_sent_cmd:
            for h in handles:
                self.dispatch_task(h, "stepper_x", "grab_start", {})
            self.has_sent_cmd = True
            self.get_logger().info("[GRABBING] 三抓手 grab_start 已下发")

        # ── 阶段 1: 等待抓取完成 (0x7C) ──
        all_done = all(
            self.world.get("handles", {}).get(h, {}).get("action_done", False)
            for h in handles
        )
        if not all_done:
            return

        # ── 阶段 2: 读颜色 → 数据融合 ──
        grabbed = {}
        for h in handles:
            grabbed[h] = self.world.get("handles", {}).get(h, {}).get("color_id", 0)
        self._data_fusion(grabbed)
        self.get_logger().info(
            f"[GRABBING] 完成: colors={grabbed}, "
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
        DROP_TRACK_POS = [5, 1, 2, 3, 4]

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

        # --- 路径规划 ---
        if self.is_ideal:
            self.drop_plan = self._plan_ideal_paths()
            self.get_logger().info(
                f"[DATA_FUSION] 理想路径规划完成: {self.drop_plan}"
            )
        else:
            self.drop_plan = self._plan_nonideal_paths()
            self.get_logger().info("[DATA_FUSION] 非理想状态，路径规划完成")

    def _check_ideal(self):
        """
        CW 顺序判定：
        从 target_L 出发顺时针走，先遇 target_M 再遇 target_R → 理想。
        否则非理想（含目标重复、目标为0、顺序错误）。
        """
        tL, tM, tR = self.target_L, self.target_M, self.target_R
        if 0 in (tL, tM, tR):
            return False

        pos = tL
        hit_M = False
        while True:
            if pos == tM:
                hit_M = True
            if pos == tR:
                return hit_M   # 遇到R时, 之前遇到过M → True, 否则 False
            pos = self._cw_next[pos]
            if pos == tL:       # 绕了一圈
                return False

    # =================================================================
    #  路径规划 — 环形轨道贪心算法
    # =================================================================

    def _pos_to_pulse(self, pos_idx):
        """POS 索引 (1~8) → 脉冲值"""
        mapping = {
            1: self.POS_1, 2: self.POS_2, 3: self.POS_3,
            4: self.POS_4, 5: self.POS_5, 6: self.POS_6, 7: self.POS_7,
            8: self.POS_8,
        }
        return mapping.get(pos_idx, 0)

    def _cw_cost(self, start, end):
        """从 start 顺时针走到 end 的代价（不含 start，含 end 的入边）"""
        cost = 0.0
        pos = start
        while pos != end:
            nxt = self._cw_next[pos]
            cost += self._edge_cost[pos][nxt]
            pos = nxt
        return cost

    def _ccw_cost(self, start, end):
        """从 start 逆时针走到 end 的代价"""
        cost = 0.0
        pos = start
        while pos != end:
            prv = self._ccw_next[pos]
            cost += self._edge_cost[pos][prv]
            pos = prv
        return cost

    def _cw_path_nodes(self, start, end):
        """顺时针路径经过的中间节点列表（不含 start，不含 end）"""
        nodes = []
        pos = start
        while pos != end:
            nxt = self._cw_next[pos]
            nodes.append(nxt)
            pos = nxt
        if nodes and nodes[-1] == end:
            nodes.pop()
        return nodes

    def _ccw_path_nodes(self, start, end):
        """逆时针路径经过的中间节点列表（不含 start，不含 end）"""
        nodes = []
        pos = start
        while pos != end:
            prv = self._ccw_next[pos]
            nodes.append(prv)
            pos = prv
        if nodes and nodes[-1] == end:
            nodes.pop()
        return nodes

    def _plan_ideal_paths(self):
        """
        贪心+10 路径规划。
        返回 drop_plan: [批次0: {hand: (pulse_target, direction, drop)}]
        理想情况只有一批，三个手各一条指令，全部 drop=True。
        """
        # AVOID_2 终点 = 状态9起点
        current = {"handle_left": 5, "handle_mid": 3, "handle_right": 6}
        targets = {
            "handle_left":  self.target_L,
            "handle_mid":   self.target_M,
            "handle_right": self.target_R,
        }

        locked = set()      # 已锁定目标点位(POS索引)
        plans = {}          # {hand: (pulse, dir)}
        hands_done = set()

        while len(hands_done) < 3:
            candidates = []
            for hand in ["handle_left", "handle_mid", "handle_right"]:
                if hand in hands_done:
                    continue
                s = current[hand]
                e = targets[hand]

                # CW 代价
                cw = self._cw_cost(s, e)
                for p in self._cw_path_nodes(s, e):
                    if p in locked:
                        cw += 10.0

                # CCW 代价
                ccw = self._ccw_cost(s, e)
                for p in self._ccw_path_nodes(s, e):
                    if p in locked:
                        ccw += 10.0

                best_cost = min(cw, ccw)
                best_dir = self.DIR_CW if cw <= ccw else self.DIR_CCW
                candidates.append((best_cost, best_dir, hand, s, e))

            # 代价最小者优先
            candidates.sort(key=lambda x: x[0])
            _, direction, hand, s, e = candidates[0]

            plans[hand] = (self._pos_to_pulse(e), direction, True)
            locked.add(e)
            hands_done.add(hand)
            current[hand] = e

        return [{hand: plans[hand] for hand in ["handle_left", "handle_mid", "handle_right"]}]

    # =================================================================
    #  路径规划 — 非理想（两批次）
    # =================================================================

    def _shift_pos(self, pos, steps, direction):
        """将 pos 沿 direction 方向移动 steps 站，返回新位置 (1~7)"""
        for _ in range(steps):
            if direction == self.DIR_CW:
                pos = self._cw_next[pos]
            else:
                pos = self._ccw_next[pos]
        return pos

    def _count_stations(self, start, end, direction):
        """计算从 start 沿 direction 到 end 需要走几站"""
        count = 0
        pos = start
        while pos != end:
            if direction == self.DIR_CW:
                pos = self._cw_next[pos]
            else:
                pos = self._ccw_next[pos]
            count += 1
        return count

    def _find_safe_pos(self, pos, blocked, target):
        """
        在未被 blocked 的节点中找一个安全位。
        优先选离 target 最近的（取 CW/CCW 中短的那条）。
        返回 POS 索引 1~7，若无安全位返回 None。
        """
        candidates = []
        for p in range(1, 9):
            if p in blocked:
                continue
            to_target = min(
                self._cw_cost(p, target),
                self._ccw_cost(p, target),
            )
            from_here = min(
                self._cw_cost(pos, p),
                self._ccw_cost(pos, p),
            )
            candidates.append((from_here + to_target, p))
        if not candidates:
            return None
        candidates.sort()
        return candidates[0][1]

    def _plan_nonideal_paths(self):
        """
        非理想路径规划：
        1. 算6个独立代价
        2. 6个两两和取最小 → 两个首批手
        3. 第三手堵了则挪到安全空位
        4. 第三手算最优方向+站数 → 全员同向同站数伴飞
        """
        hands = ["handle_left", "handle_mid", "handle_right"]
        current = {"handle_left": 5, "handle_mid": 3, "handle_right": 6}
        targets = {
            "handle_left":  self.target_L,
            "handle_mid":   self.target_M,
            "handle_right": self.target_R,
        }

        # ---- Step 1: 6 个独立代价 ----
        costs = {}
        for h in hands:
            s, e = current[h], targets[h]
            costs[h] = {
                self.DIR_CW:  self._cw_cost(s, e),
                self.DIR_CCW: self._ccw_cost(s, e),
            }

        # ---- Step 2: 6 个同向两两和，排序 ----
        pairs = []
        for i, h1 in enumerate(hands):
            for h2 in hands[i + 1:]:
                for d in (self.DIR_CW, self.DIR_CCW):
                    total = costs[h1][d] + costs[h2][d]
                    pairs.append((total, h1, d, h2, d))
        pairs.sort(key=lambda x: x[0])

        # ---- 逐个尝试，找可行解 ----
        for _, h1, d1, h2, d2 in pairs:
            h3 = [h for h in hands if h != h1 and h != h2][0]

            # batch0 两个手的路径和目标
            path1 = self._cw_path_nodes(current[h1], targets[h1]) if d1 == self.DIR_CW \
                else self._ccw_path_nodes(current[h1], targets[h1])
            path2 = self._cw_path_nodes(current[h2], targets[h2]) if d2 == self.DIR_CW \
                else self._ccw_path_nodes(current[h2], targets[h2])
            occupied = set(path1 + path2 + [targets[h1], targets[h2]])

            # ---- Step 3: 第三手堵塞检查 ----
            batch0 = {}
            post_current = {h1: targets[h1], h2: targets[h2]}

            if current[h3] in occupied:
                safe = self._find_safe_pos(current[h3], occupied, targets[h3])
                if safe is None:
                    continue   # 无安全位，试下一对
                # 给 h3 找最短路径去 safe
                cw_to = self._cw_cost(current[h3], safe)
                ccw_to = self._ccw_cost(current[h3], safe)
                safe_dir = self.DIR_CW if cw_to <= ccw_to else self.DIR_CCW
                batch0[h3] = (self._pos_to_pulse(safe), safe_dir, False)
                post_current[h3] = safe
                self.get_logger().info(
                    f"[NONIDEAL] {h3} 堵在{current[h3]}, 挪到安全位{safe}"
                )
            else:
                post_current[h3] = current[h3]

            batch0[h1] = (self._pos_to_pulse(targets[h1]), d1, True)
            batch0[h2] = (self._pos_to_pulse(targets[h2]), d2, True)

            # ---- Step 4: 第三手去目标 + 全员伴飞 ----
            h3_cw = self._cw_cost(post_current[h3], targets[h3])
            h3_ccw = self._ccw_cost(post_current[h3], targets[h3])
            h3_dir = self.DIR_CW if h3_cw <= h3_ccw else self.DIR_CCW
            h3_stations = self._count_stations(post_current[h3], targets[h3], h3_dir)

            batch1 = {}
            for h in hands:
                new_pos = self._shift_pos(post_current[h], h3_stations, h3_dir)
                is_drop = (h == h3)
                batch1[h] = (self._pos_to_pulse(new_pos), h3_dir, is_drop)

            return [batch0, batch1]

        return []   # 兜底：理论上不会到这里

    # =================================================================
    #  状态 5：HANDS_TO_AVOID_1 — 抓手→避障区1
    # =================================================================

    def _handle_hands_avoid1(self):
        """左→8(CCW), 中→1(CCW), 右→4(CCW) — plan_readme 4.7"""
        if not self.has_sent_cmd:
            self._move_hand_to("handle_left",  8, self.DIR_CCW)
            self._move_hand_to("handle_right", 4, self.DIR_CCW)
            self._move_hand_to("handle_mid",   1, self.DIR_CCW)
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
        """左→5(CW), 中→3(CW), 右→6(CW) — plan_readme 4.9"""
        if not self.has_sent_cmd:
            self._move_hand_to("handle_left",  5, self.DIR_CW)
            self._move_hand_to("handle_right", 6, self.DIR_CW)
            self._move_hand_to("handle_mid",   3, self.DIR_CW)
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
            self._drop_batch = 0
            self._drop_step = 0
            self._drop_step_timer = 0
            self.get_logger().info("[EXECUTE_TARGET] 进入放豆执行状态")
            self.has_sent_cmd = True

        if not self._execute_done:
            self.execute_drop()

        if self._execute_done:
            self.get_logger().info("[EXECUTE_TARGET] 执行完毕，进入 CHASSIS_TO_END")
            self._transition_to(self.ST_CHASSIS_TO_END)

    # =================================================================
    #  execute_drop — 批次驱动放豆执行
    # =================================================================

    def _hands_arrived(self, hands):
        """检查指定 hands 是否全部 track_arrived"""
        return all(
            self.world.get("handles", {}).get(h, {}).get("track_arrived", False)
            for h in hands
        )

    def execute_drop(self):
        """
        按 drop_plan 逐批执行。
        batch 条目: {hand: (pulse_target, direction, drop_flag)}
        流程: X移动 → 等到位 → 舵机转90°放豆 → 下一批。
        只有 drop_flag=True 的手才触发舵机。
        """
        if self._drop_batch >= len(self.drop_plan):
            self._execute_done = True
            return

        batch = self.drop_plan[self._drop_batch]
        hands = list(batch.keys())
        droppers = [h for h in hands if batch[h][2]]

        # ──── step 0: 下发 X 轴移动 ────
        if self._drop_step == 0:
            for hand in hands:
                target, direction, _ = batch[hand]
                self._move_hand_x(hand, target, direction)
                self.get_logger().info(
                    f"[DROP] 批次{self._drop_batch} {hand} → pos={target} dir={direction}"
                )
            self._drop_step = 1

        # ──── step 1: 等待 X 轴到位 ────
        elif self._drop_step == 1:
            if self._hands_arrived(hands):
                if droppers:
                    self.get_logger().info(
                        f"[DROP] 批次{self._drop_batch} X轴到位，放豆 "
                        f"(放豆手: {droppers})"
                    )
                    self._drop_step = 2
                    self._drop_step_timer = 0
                else:
                    self.get_logger().info(
                        f"[DROP] 批次{self._drop_batch} X轴到位，无放豆手，跳过"
                    )
                    self._drop_batch += 1
                    self._drop_step = 0

        # ──── step 2: 舵机转 90° 放豆 (消抖 5 周期) ────
        elif self._drop_step == 2:
            if self._drop_step_timer == 0:
                for hand in droppers:
                    self.dispatch_task(hand, "servo", "move_to", {"angle": 90})
                self.get_logger().info(
                    f"[DROP] 批次{self._drop_batch} 舵机张开 90°"
                )
            self._drop_step_timer += 1
            if self._drop_step_timer >= 5:
                self.get_logger().info(
                    f"[DROP] 批次{self._drop_batch} 完成"
                )
                self._drop_batch += 1
                self._drop_step = 0

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
    #  ST_RESET — 复位: 底盘回0 + 抓手回位 + 编码器清零
    # =================================================================

    def _handle_reset(self):
        """复位流程: 并发移动 → 等到齐 → 清零编码器"""
        # ── 阶段 0: 发移动指令 ──
        if not self.has_sent_cmd:
            # 底盘回起始区
            self.dispatch_task("chassis", "base", "move_to",
                               {"pos": self.POS_START_ZONE})
            # 三抓手固定方向回位
            self._move_hand_to("handle_left",  1, self.DIR_CW)   # 顺时针
            self._move_hand_to("handle_mid",   2, self.DIR_CCW)  # 逆时针
            self._move_hand_to("handle_right", 3, self.DIR_CCW)  # 逆时针
            self.has_sent_cmd = True
            self.get_logger().info("[RESET] 4路并行回位指令已下发")

        # ── 阶段 1: 等待全部到齐 ──
        chassis_ok = self.world.get("chassis", {}).get("arrival_done", False)
        handles = self.world.get("handles", {})
        hands_ok = all(
            handles.get(h, {}).get("track_arrived", False)
            for h in ["handle_left", "handle_mid", "handle_right"]
        )
        if not (chassis_ok and hands_ok):
            return

        # ── 阶段 2: 三抓手编码器清零 (X轴+Z轴) ──
        self.get_logger().info("[RESET] 全部到齐, 编码器清零")
        for h in ["handle_left", "handle_mid", "handle_right"]:
            self.dispatch_task(h, "stepper_x", "reset_encoder", {})
            self.dispatch_task(h, "stepper_z", "reset_encoder", {})

        # ── 完成 → 解锁 + 单步 + 切 INIT ──
        self.estop_locked = False
        self.step_mode = True
        self.step_paused = True
        self._transition_to(self.ST_INIT)
        self.get_logger().info("[RESET] 复位完成 → INIT (已解锁, 单步模式)")

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

    def _move_hand_to(self, hand, pos_id, direction):
        """track_move(0x7A): 环轨点位移动, pos_id 1~8"""
        clockwise = 1 if direction == self.DIR_CW else 0
        self.dispatch_task(hand, "stepper_x", "track_move",
                           {"pos_id": pos_id, "clockwise": clockwise})

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
            self.POS_8 = r["pos_8"]

            # --- 底盘目标位置 ---
            c = cfg["chassis"]
            self.POS_START_ZONE = c["start_zone"]
            self.POS_GRAB_ZONE = c["grab_zone"]
            self.POS_DROP_ZONE = c["drop_zone"]
            self.POS_END_ZONE = c["end_zone"]
            self.POS_OBSTACLE_A = c["obstacle_a"]
            self.POS_OBSTACLE_B = c["obstacle_b"]

            self.get_logger().info(f"Loaded position params from {config_path}")

        except Exception as e:
            self.get_logger().error(
                f"Failed to load {config_path}: {e}. Using defaults."
            )
            self._set_default_positions()

    def _set_default_positions(self):
        """硬编码默认值（YAML 加载失败时的兜底）"""
        self.RING_MAX = 141303
        self.POS_1 = 0
        self.POS_7 = 14022
        self.POS_2 = 14382
        self.POS_3 = 28764
        self.POS_4 = 46741
        self.POS_8 = 64000
        self.POS_6 = 101393
        self.POS_5 = 123325

        self.POS_START_ZONE = 0
        self.POS_GRAB_ZONE = -48114
        self.POS_DROP_ZONE = 30375
        self.POS_END_ZONE = 0
        self.POS_OBSTACLE_A = -40095
        self.POS_OBSTACLE_B = 27945

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
            12: "RESET",
        }
        return names.get(self.state, f"UNKNOWN({self.state})")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(BrainNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
