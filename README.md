# Rosmaster
2026年中国大学生机械工程创新创意大赛

src/
    ├── robot_bringup/       # 启动中心：存放所有 launch 文件和配置文件
    ├── robot_description/   # 模型中心：存放 URDF 模型、坐标系 (TF) 定义
    ├── robot_interfaces/    # 自定义接口：存放特殊的串口通信协议或传感器消息
    ├── robot_base/          # 底盘驱动：即你现在的串口通信节点
    ├── robot_sensors/       # 传感器驱动：后续的雷达、摄像头驱动配置
    └── robot_navigation/    # 导航配置：存放 Nav2 的参数文件和地图robot_navigation