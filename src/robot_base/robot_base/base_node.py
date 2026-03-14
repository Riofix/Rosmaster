#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
import rclpy
from rclpy.node import Node

# 导入 ROS 2 标准消息
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

# 导入我们的纯净驱动层
from .serial_driver import ChassisDriver

def euler_to_quaternion(yaw, pitch=0.0, roll=0.0):
    """简单的欧拉角转四元数辅助函数，避免引入额外依赖"""
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class BaseNode(Node):
    def __init__(self):
        super().__init__('robot_base_node')
        
        # 1. 声明和获取 ROS 参数 (端口号、波特率、坐标系名称)
        self.declare_parameter('port', '/dev/rosmaster')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        
        port = self.get_parameter('port').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # 2. 里程计推算状态变量
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_th = 0.0
        self.last_time = self.get_clock().now()
        self.odom_lock = threading.Lock() # 保护里程计数据的线程锁

        # 3. 初始化 ROS 发布者和 TF 广播者
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 4. 初始化 ROS 订阅者 (键盘或导航栈发来的速度指令)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # 5. 初始化底层驱动，并传入回调函数处理底盘发来的数据
        self.driver = ChassisDriver(port=port, baudrate=115200, data_callback=self.on_chassis_data)
        if not self.driver.connect():
            self.get_logger().error(f"无法打开串口 {port}，请检查权限或连接！")
            # 实际应用中这里可以抛出异常或重试
        else:
            self.get_logger().info(f"底盘连接成功，准备就绪。通信端口: {port}")

    def cmd_vel_callback(self, msg: Twist):
        """处理下发的 cmd_vel 速度指令"""
        vx = msg.linear.x
        vy = msg.linear.y   # 如果是全向轮小车(麦轮)，Y轴不为0
        vth = msg.angular.z
        
        # 直接调用驱动层发送，完全屏蔽十六进制协议
        self.driver.set_velocity(vx, vy, vth)

    def on_chassis_data(self, data_dict):
        """
        底层驱动层解析完数据的回调函数。
        注意：这个函数是在驱动的后台读取线程中被触发的。
        """
        if data_dict.get('type') == 'motion_data':
            # 提取底盘实时反馈的实际速度
            vx = data_dict['linear_x']
            vy = data_dict['linear_y']
            vth = data_dict['angular_z']
            
            # 计算时间差 (dt)
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time

            # 里程计积分推算坐标 (x, y, theta)
            with self.odom_lock:
                delta_x = (vx * math.cos(self.odom_th) - vy * math.sin(self.odom_th)) * dt
                delta_y = (vx * math.sin(self.odom_th) + vy * math.cos(self.odom_th)) * dt
                delta_th = vth * dt

                self.odom_x += delta_x
                self.odom_y += delta_y
                self.odom_th += delta_th
                
                # 发布 Odom 和 TF
                self._publish_odom(vx, vy, vth, current_time)

    def _publish_odom(self, vx, vy, vth, current_time):
        """组装并发布 TF 和 Odometry 消息 (满足导航需求)"""
        q = euler_to_quaternion(self.odom_th)
        msg_time = current_time.to_msg()

        # 1. 发布 TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = msg_time
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = self.odom_x
        t.transform.translation.y = self.odom_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # 2. 发布 Odom 消息
        odom = Odometry()
        odom.header.stamp = msg_time
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # 设置位置
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # 设置速度
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        
        self.odom_pub.publish(odom)

    def destroy_node(self):
        """节点销毁时，确保安全断开硬件"""
        if hasattr(self, 'driver'):
            # 停止小车
            self.driver.set_velocity(0.0, 0.0, 0.0)
            self.driver.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('检测到 Ctrl+C，正在安全关闭底盘节点...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()