std_msgs/Header header

# 1. 底盘大盘状态
float32 chassis_x
float32 chassis_y
float32 chassis_yaw
float32 battery_voltage

# 2. 从机吊具集合状态
robot_interfaces/CraneState crane_left
robot_interfaces/CraneState crane_middle
robot_interfaces/CraneState crane_right

# 3. 全局视野关注目标
robot_interfaces/DetectionList visible_targets

# 4. 当前系统行为流状态 (预留给行为树)
uint8 SYSTEM_IDLE = 0
uint8 SYSTEM_SEARCHING = 1
uint8 SYSTEM_ALIGNING = 2
uint8 SYSTEM_GRABBING = 3
uint8 SYSTEM_ERROR = 99
uint8 current_state
