# 从机吊具由于带有各种异步硬件，所有状态应当被集中汇总报告

# 携带时间戳等多重标准头信息
std_msgs/Header header

# ------ 步进电机里程与闭环数据 ------
float32 track_pos      # EMM电机 1: 轨道车在环形导轨上的当前水平里程 (单位可由您在协议解析层换算，如米或毫米)
float32 hoist_depth    # EMM电机 2: 抓手吊具的当前下达深度

# ------ MPU6050轨道姿态补偿数据 ------
float32 carriage_yaw   # 轨道车偏航角 (核心: 用于进入环形弯道特征点时做里程补偿)
float32 carriage_roll  # 轨道车侧倾角
float32 carriage_pitch # 轨道车俯仰角

# ------ 末端执行器 (吸尘器与出料口) ------
int8 vacuum_power      # 无刷吸尘电机的占空比 (0为关闭，100为满载)
bool is_outlet_open    # 控制出料舱的舵机状态 (False: 闭合, True: 打开)
