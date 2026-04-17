#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from robot_interfaces.msg import Detection, DetectionList

class DualCamNode(Node):
    """
    负责驱动外置 USB 双目相机的机器视觉感知节点
    不包含底层通信的噪音，专门进行 OpenCV 画面的二值化与数字特征过滤
    输出结构化的高层次感知数据 (Bounding Box + Label)。
    """
    def __init__(self):
        super().__init__('dual_cam_node')
        
        # 定义相机索引（需要根据实际的 v4l2 挂载点进行重定向）
        self.declare_parameter('cam_left_id', 0)
        self.declare_parameter('cam_right_id', 1)
        # 为节约算力，默认降低相机解析帧率进行 OpenCV 洗理
        self.declare_parameter('fps', 10) 
        
        c1 = self.get_parameter('cam_left_id').value
        c2 = self.get_parameter('cam_right_id').value
        fps = self.get_parameter('fps').value
        
        self.get_logger().info(f"正在初始化 OpenCV 双目捕获引擎, 左相机:{c1}, 右相机:{c2}")
        
        # 尝试接入底层视频流 (Windows 测试时不报错)
        self.cap_left = cv2.VideoCapture(c1)
        self.cap_right = None # 如果只有一个相机，可以将此处修改或者根据设备尝试打开
        if self.cap_left.isOpened():
            self.get_logger().info(f"左眼 (ID:{c1}) 已打通。")
            self.cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        else:
            self.get_logger().error(f"打死也连不上左眼 (ID:{c1}) ！请检查 /dev/video*")

        self.cap_right = cv2.VideoCapture(c2)
        if self.cap_right.isOpened():
            self.get_logger().info(f"右眼 (ID:{c2}) 已打通。")
            self.cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        else:
            self.get_logger().warn(f"无法连上右眼 (ID:{c2})，切入单目备用模式运行。")
            
        self.bridge = CvBridge()
        
        # 感知结果播报 (供 Fusion 数据层索取)
        self.det_pub = self.create_publisher(DetectionList, '/vision/detections', 10)
        
        # [可选] 发送识别后的画面，供开发者在 Rviz2 调试分析 (极度消耗内存带宽，不建议高频)
        self.img_pub = self.create_publisher(Image, '/vision/debug_view', 5)

        # 构建主处理定时器
        period = 1.0 / fps
        self.timer = self.create_timer(period, self.vision_pipeline_callback)

    def extract_digits(self, frame):
        """
        核心 OpenCV 图像解析管道
        您需要根据起重机的抓取环境，调整阈值和形态学操作
        """
        detections = []
        
        # 1. 灰度化 + 高斯滤波降噪
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # 2. 自适应二值化 或 阈值提取 (假设数字和底板对比度高)
        _ , thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)
        
        # 3. 寻找外轮廓
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500 and area < 20000: # 过滤极小噪点和极大错误块 (依据实物体积调整)
                x, y, w, h = cv2.boundingRect(cnt)
                
                # 过滤狭长比的干扰线形物体
                aspect_ratio = float(w)/h
                if 0.3 < aspect_ratio < 3.0: 
                    # 记录并提取 ROI，方便进行模板匹配或 CNN 识别
                    # 针对您的设计：可以在这里嵌套一个简单的模板匹配或者 pytesseract 获取识别的数字
                    # 假定目前它找到了一个感兴趣的目标 (未知数字物体)
                    det = Detection()
                    det.label = "unknown"
                    det.confidence = 0.9
                    det.pixel_x = x + w//2
                    det.pixel_y = y + h//2
                    detections.append((det, (x, y, w, h)))
                    
        return detections, thresh
        
    def vision_pipeline_callback(self):
        msg = DetectionList()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link_combined"
        
        combined_debug = None
        
        # ============ 执行左眼推理 ============
        if self.cap_left and self.cap_left.isOpened():
            ret, frame1 = self.cap_left.read()
            if ret:
                dets, debug_bin1 = self.extract_digits(frame1)
                
                # 可视化标注:
                debug_draw = frame1.copy()
                for det, bbox in dets:
                    # 附加属性说明此特征提取自相机1
                    det.label = f"L_{det.label}"
                    msg.detections.append(det)
                    x, y, w, h = bbox
                    cv2.rectangle(debug_draw, (x,y), (x+w,y+h), (0,255,0), 2)
                    cv2.putText(debug_draw, det.label, (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                
                combined_debug = debug_draw
        
        # ============ 执行右眼推理 ============
        if self.cap_right and self.cap_right.isOpened():
            ret, frame2 = self.cap_right.read()
            if ret:
                dets, debug_bin2 = self.extract_digits(frame2)
                
                debug_draw2 = frame2.copy()
                for det, bbox in dets:
                    det.label = f"R_{det.label}"
                    msg.detections.append(det)
                    x, y, w, h = bbox
                    cv2.rectangle(debug_draw2, (x,y), (x+w,y+h), (0,0,255), 2)
                    cv2.putText(debug_draw2, det.label, (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
                
                if combined_debug is not None:
                    # 横向拼接双目画面方便调试
                    combined_debug = np.hstack((combined_debug, debug_draw2))
                else:
                    combined_debug = debug_draw2
                    
        # ============ 结果上报数据层 ============
        if len(msg.detections) > 0:
            self.det_pub.publish(msg)
            
        # ============ 可视化推送 ============
        if combined_debug is not None and self.img_pub.get_subscription_count() > 0:
            try:
                # 只有当有人真的在使用 Rviz2 查看时，才把 NumPy 转成图片发出去，狠狠省 CPU
                img_msg = self.bridge.cv2_to_imgmsg(combined_debug, encoding="bgr8")
                self.img_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"Image 转换抛锚了: {e}")

    def destroy_node(self):
        if self.cap_left:
            self.cap_left.release()
        if self.cap_right:
            self.cap_right.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DualCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
