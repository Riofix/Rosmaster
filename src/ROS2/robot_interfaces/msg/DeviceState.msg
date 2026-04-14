# 设备网络与心跳状态反馈

# 设备名，例如 "crane_left", "crane_middle", "crane_right" 或 "chassis"
string device_name

# 设备在局域网内挂载的 IP（只针对 ESP8266有效，主底盘串口为空）
string ip_address

# 设备健康枚举状态
uint8 STATUS_OFFLINE = 0   # 设备离线超时、掉线脱链
uint8 STATUS_IDLE = 1      # 连入网络且空闲中
uint8 STATUS_WORKING = 2   # 正在执行搬运任务中
uint8 STATUS_ERROR = 3     # 在线但底层报出硬件故障（如电机堵转异常返回）

uint8 current_status       # 设备目前实时状态
