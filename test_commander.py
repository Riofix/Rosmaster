#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 测试上位机发送指令到起重机及底盘
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import CraneTrigger
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray
import time

class TestCommander(Node):
    def __init__(self):
        super().__init__('test_commander')
        
        # 吊臂类控制句柄
        self.cli_left = self.create_client(CraneTrigger, '/crane_left/trigger')
        self.step_left_pub = self.create_publisher(Float32MultiArray, '/crane_left/debug_step', 10)
        
        # 底盘类控制句柄
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.beep_pub = self.create_publisher(Bool, '/chassis/cmd_beep', 10)
        
    def trigger_crane(self, t_type, val):
        req = CraneTrigger.Request()
        req.trigger_type = t_type
        req.target_value = val
        self.get_logger().info("等待 tcp_server 与 ESP8266 专线连通...")
        self.cli_left.wait_for_service()
        self.get_logger().info(f"发出触发信号: 类型={t_type}, 参数={val}")
        return self.cli_left.call_async(req)

    def move_stepper(self, addr, direction, vel, ticks):
        msg = Float32MultiArray()
        # [地址(1/2), 转向(0/1), 速度, 行驶脉冲刻度]
        msg.data = [float(addr), float(direction), float(vel), float(ticks)]
        self.step_left_pub.publish(msg)
        self.get_logger().info(f"发送步进跑动要求 => 地址:{addr} 脉冲数:{ticks}")

    def move_chassis(self, vx, vth):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(vth)
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f"发送底盘行驶求 => Vx: {vx}m/s  Vth: {vth}rad/s")
        
    def beep_chassis(self):
        msg = Bool()
        msg.data = True
        self.beep_pub.publish(msg)
        self.get_logger().info("发送底盘鸣笛指令 500ms")

def main():
    rclpy.init()
    node = TestCommander()
    
    while rclpy.ok():
        print("\n==================================")
        print("  全体系 ROS2 机器人物理组件试车台  ")
        print("==================================")
        print("-----------  吊臂末端工具类 -----------")
        print(" 1 : 左吊具 - 无刷电机吸尘器 [满功率吸附]")
        print(" 2 : 左吊具 - 无刷电机吸尘器    [关闭]")
        print(" 3 : 左吊具 - 舵机出料舱门      [打开]")
        print(" 4 : 左吊具 - 舵机出料舱门      [闭合]")
        print("----------- 吊臂步进移动类 -----------")
        print(" 5 : 左吊具 - 轨道车 (地址1) 相对走 2000 步")
        print(" 6 : 左吊具 - 升降轴 (地址2) 相对降 1000 步")
        print("----------- 底盘运动警报类 -----------")
        print(" 7 : 主底盘 - 前进 (Vx = 0.2 m/s)")
        print(" 8 : 主底盘 - 旋转 (Vth = +0.5 rad/s)")
        print(" 9 : 主底盘 - 急停 (0 m/s)")
        print(" B : 主底盘 - 发出哔哔声 (喇叭)")
        print(" q : 退出")
        print("==================================")
        
        try:
            choice = input("请输入测试指令代号 (1-9, B, q) >> ")
            if choice == '1':
                future = node.trigger_crane(CraneTrigger.Request.TRIGGER_VACUUM, 100)
                rclpy.spin_until_future_complete(node, future)
            elif choice == '2':
                future = node.trigger_crane(CraneTrigger.Request.TRIGGER_VACUUM, 0)
                rclpy.spin_until_future_complete(node, future)
            elif choice == '3':
                future = node.trigger_crane(CraneTrigger.Request.TRIGGER_OUTLET, 100) # 假定100为开角
                rclpy.spin_until_future_complete(node, future)
            elif choice == '4':
                future = node.trigger_crane(CraneTrigger.Request.TRIGGER_OUTLET, 0)   # 假定0为闭角
                rclpy.spin_until_future_complete(node, future)
            elif choice == '5':
                # 地址1, 顺时针0, 速度500, 脉冲2000
                node.move_stepper(1, 0, 500, 2000) 
            elif choice == '6':
                # 地址2, 逆时针1, 速度500, 脉冲1000
                node.move_stepper(2, 1, 500, 1000)
            elif choice == '7':
                node.move_chassis(0.2, 0.0)
            elif choice == '8':
                node.move_chassis(0.0, 0.5)
            elif choice == '9':
                node.move_chassis(0.0, 0.0)
            elif choice.lower() == 'b':
                node.beep_chassis()
            elif choice.lower() == 'q':
                break
        except KeyboardInterrupt:
            break
            
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
