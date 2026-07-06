# ROS2 上位机改造计划

## 目标

适配 STM32 固件 v2.0 新增指令，简化下行路三个节点 (protocol_pack / control / brain)。

## 背景

STM32 固件新增 `app_action.c`：

| 指令 | 功能 | 上位机只需发 |
|------|------|-------------|
| 0x78 | CM 位置控制 | dist_cm × 100 |
| 0x79 | 抓取序列(11步) | 一个字节 |
| 0x7A | 环轨点位移动 | pos_id + dir |
| 0x7B | 校准原点 | pos_id |
| 0x7C | 抓取完成通知(TX) | STM32→上位机 |

---

## 执行顺序

| # | 模块 | 状态 |
|------|------|------|
| 1 | protocol_pack_node | ✅ |
| 2 | control_node | ✅ |
| 3 | 0x7C 抓取完成通知 (固件 + protocol_node) | ✅ |
| 4 | brain_node 重构 | ✅ |

---

## 1. protocol_pack_node.py ✅

新增 0x78 / 0x79 / 0x7A / 0x7B 四条指令封包。

## 2. control_node.py ✅

- cmd_map 加 `move_cm:0x78` / `grab_start:0x79` / `track_move:0x7A` / `set_origin:0x7B`
- brain_cb 加 grab_start / track_move / set_origin 三个直接封包分支

## 3. 0x7C 抓取完成通知 ✅

| 文件 | 改动 |
|------|------|
| `app_action.c` | Grab 回 IDLE 时 `Protocol_PackAndSend` 发 `[0x7C]` |
| `cmd_handle.h` | 加 `CMD_TX_ACTION_DONE = 0x7C` |
| `protocol_node.py` | 解析表加 `0x7C: action_done = True` |

## 4. brain_node ⏳

### 4.1 环形轨道新版

**顺时针序**: `1→2→7→3→4→8→6→5→(回1)`

| 编号 | 用途 |
|------|------|
| 1 | 放豆/放料 |
| 2 | 放豆 |
| 3 | 放豆 |
| 4 | 放豆 |
| 5 | 放豆 |
| 6 | 左抓手抓取位 |
| 7 | 中抓手抓取位 (新增) |
| 8 | 右抓手抓取位 |

**edge_cost**:
```python
_edge_cost = {
    1: {2: 0.2, 5: 0.1},
    2: {1: 0.2, 7: 0.01, 3: 0.2},
    7: {2: 0.01, 3: 0.19},
    3: {7: 0.19, 2: 0.2, 4: 0.1},
    4: {3: 0.1, 8: 0.2},
    8: {4: 0.2, 6: 0.25},
    6: {8: 0.25, 5: 0.2},
    5: {6: 0.2, 1: 0.1},
}
```

**DROP_TRACK_POS**: `[5, 1, 2, 3, 4]`

**放豆角度**: 0°=开(放豆), 90°=关

### 4.2 ST_INIT — 初始化 (10 步)

| 步骤 | 动作 | 指令/检测 |
|------|------|------|
| 0 | 设备连接检查 | 读 world_state, 3 TCP + 底盘 串口在线 |
| 0.5 | 开启自动上报 | 三抓手各发 0x72/0x73/0x74 |
| 1-3 | 校准原点 | set_origin(左=1, 中=2, 右=3) |
| 4 | 电机2上升至堵转 | velocity(2, dir=0), 检测 flag & 0x04 |
| 5 | 停止电机2 | 0x63 |
| 6 | 电机2回零 | 0x6A |
| 7 | 解除堵转保护 | 0x6B |
| 8 | 舵机闭合 | servo move_to(90) × 3 |
| 9 | 获取视觉序列 | 读 vision.sequence |
| 10 | → ST_WAIT_START_CMD | 跳过 ST_WAIT_VISION |

### 4.3 ST_WAIT_VISION — 跳过

保留代码，状态转移直接从 INIT → ST_WAIT_START_CMD。

### 4.4 ST_WAIT_START_CMD — 不变

等待外部 `/task_control {"cmd":"start"}` → ST_MOVE_TO_GRAB_ZONE。

### 4.5 ST_MOVE_TO_GRAB_ZONE

| 抓手 | 起点 | 目标 | 方向 |
|------|------|------|------|
| 左 | 1 | 6 | 逆时针 |
| 右 | 3 | 8 | 顺时针 |
| 中(第1步) | 2 | 3 | 顺时针 |
| 中(第2步) | 3 | 7 | 逆时针 (避障点A后) |

全部到位 → ST_GRABBING

### 4.6 ST_GRABBING

1. 三抓手同时 `grab_start` (0x79)
2. 等 0x7C `action_done`
3. 读 color_id → 数据融合 → 判定理想/非理想
4. `grab_bean()` 整个删除
5. → ST_HANDS_TO_AVOID_1

### 4.7 ST_HANDS_TO_AVOID_1

起点: 左6, 中7, 右8

| 抓手 | 目标 | 方向 |
|------|------|------|
| 左 | 8 | 逆时针 |
| 中 | 1 | 逆时针 |
| 右 | 4 | 逆时针 |

全部到位 → ST_CHASSIS_TO_START

### 4.8 ST_CHASSIS_TO_START — 不变

底盘 → 起始区, 等到位 → ST_HANDS_TO_AVOID_2

### 4.9 ST_HANDS_TO_AVOID_2

起点: 左8, 中1, 右4

| 抓手 | 目标 | 方向 |
|------|------|------|
| 左 | 5 | 顺时针 |
| 中 | 3 | 顺时针 |
| 右 | 6 | 顺时针 |

全部到位 → ST_CHASSIS_TO_DROP

### 4.10 ST_CHASSIS_TO_DROP — 不变

→ ST_EXECUTE_TARGET

### 4.11 ST_EXECUTE_TARGET — 算法保留

按 drop_plan 逐批执行放豆 (舵机 0°)。路径规划算法逻辑不变，适配 8 节点拓扑。

### 4.12 ST_CHASSIS_TO_END — 不变

### 4.13 ST_DONE — 不变

### 4.14 代码级改动

- `_load_positions` / `_set_default_positions`: POS_1~POS_7 → POS_1~POS_8
- `_pos_to_pulse`: 1~7 → 1~8
- `_check_ideal`: `range(1,8)` + `pos = pos+1 if pos<8 else 1`
- `_cw_cost` / `_ccw_cost` / `_cw_path_nodes` / `_ccw_path_nodes` / `_shift_pos` / `_count_stations` / `_find_safe_pos`: 循环边界 7→8
- `grab_bean()` → 删除, 替换为 `dispatch_task(hand, "stepper_x", "grab_start", {})`
- `_move_hand_x()` → 替换为 `dispatch_task(hand, "stepper_x", "track_move", {"pos_id": idx, "clockwise": dir})`
- `_ring_displacement()` → 删除
- `_plan_ideal_paths` / `_plan_nonideal_paths`: 起点坐标更新

---

## 5. 已完成修改 (2026-07-06)

### Phase 1: 补齐缺失指令 ✅
- control_node: +stream子系统, +handle_stream cmd_map, +reset_clog(0x6B)
- protocol_pack: +0x6A/0x6B/0x72/0x73/0x74/0x7D 封包

### Phase 2: Bug修复 ✅
- INIT→ST_WAIT_START_CMD (跳过WAIT_VISION)
- reset cmd删estop_locked检查

### Phase 3: INIT重写 ✅
- 步骤0~10 per plan_readme 4.2
- 新增 init_step / init_step_cmd_sent 状态变量

### Phase 4: 轨道7→8 ✅
- position.yaml: +pos_8
- edge_cost: 8节点拓扑
- DROP_TRACK_POS: [5,1,2,3,4]
- 全部7→8边界更新
- AVOID_1/2状态更新
- MOVE_TO_GRAB目标更新
- _plan_ideal/_nonideal起点更新

---

## 6. 待确认问题

### Q1: INIT步骤4堵转检测
步骤4对三抓手同时发 velocity(Z轴, dir=0), 等待 flag & 0x04。
- flag & 0x04 确认是堵转标志位吗?
- 三抓手Z轴行程一致吗? 堵转位置能正常触发吗?


### Q2: 自动上报指令 (0x72/0x73/0x74)
这些指令在新版下位机固件中是否还保留? 如果下位机默认已经开启上报, 
步骤0.5可以跳过。

### Q3: pos_1~pos_8 脉冲值
当前为占位值(0,150,300,...), 需要实际校准。
环轨 pulse/mm 换算关系是什么? ring_max=178000 是否对应一整圈?

### Q4: GRABBING状态
grab_bean() 尚未替换为 grab_start(0x79)。plan_readme 4.6 要求删除 grab_bean,
改用下位机 0x79 抓取序列。但现有 grab_bean 逻辑复杂, 需要单独处理。

### Q5: _move_hand_x 替换
plan_readme 4.14 要求改为 track_move(0x7A), 传 pos_id 而非脉冲值。
当前 _move_hand_x 读编码器→算位移→发 move_relative(0x62)。
替换需要确保下位机 0x7A 正确工作。
