#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, Float32MultiArray
from robot_interfaces.srv import CraneTrigger
from .protocol_core import CraneCodec

class CranePackerNode(Node):
    """
    负责单一吊臂业务信号与 0xFF 字节流之间的单向包装
    可通过参数重定向绑定到物理 Crane 上，无需修改业务内蕴代码
    """
    def __init__(self):
        super().__init__('crane_packer_node')
        
        # 获取要效忠的主子参数 (左边/右边/中间?)
        self.declare_parameter('crane_id', 'crane_left')
        self.crane_id = self.get_parameter('crane_id').value

        # 给 Link 层的出水管
        self.tx_pub = self.create_publisher(UInt8MultiArray, f'/{self.crane_id}/tcp_tx_raw', 50)
        
        # 面向上层的瞬动触发业务接口
        self.create_service(CraneTrigger, f'/{self.crane_id}/trigger', self.trigger_cb)
        
        # 暴露出一个临时的底层调试口用来测试 EMM 步进电机
        self.create_subscription(Float32MultiArray, f'/{self.crane_id}/debug_step', self.step_cb, 10)
        self.get_logger().info(f"指令封包投递员已就绪，当前管辖下位机: {self.crane_id}")

    def step_cb(self, msg: Float32MultiArray):
        # 约定的格式: data = [addr, dir_byte, vel, target_ticks]
        if len(msg.data) >= 4:
            addr = int(msg.data[0])
            direction = int(msg.data[1])
            vel = int(msg.data[2])
            target = int(msg.data[3])
            acc = 50 # 默认加速度参数
            # 采用相对坐标(True)移动
            byte_frame = CraneCodec.pack_emm_pos_ctrl(addr, direction, vel, acc, target, True)
            
            # 发射封包
            out_msg = UInt8MultiArray()
            out_msg.data = list(byte_frame)
            self.tx_pub.publish(out_msg)

    def trigger_cb(self, request, response):
        """解析 ROS 标准 Service 参数，打包硬件专用十六进制"""
        if request.trigger_type == CraneTrigger.Request.TRIGGER_VACUUM:
            byte_frame = CraneCodec.pack_vacuum_duty(request.target_value)
            
        elif request.trigger_type == CraneTrigger.Request.TRIGGER_OUTLET:
            # 假定舵机使用通道 1 
            byte_frame = CraneCodec.pack_outlet_servo(1, request.target_value)
            
        else:
            response.success = False
            response.message = f"未知的请求触发类型 {request.trigger_type}"
            return response
            
        # 塞入 link 盲送管道
        msg = UInt8MultiArray()
        msg.data = list(byte_frame)
        self.tx_pub.publish(msg)
        
        self.get_logger().info(f"已将业务触发转译下发: 指令长度 {len(byte_frame)} Byte")
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CranePackerNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
