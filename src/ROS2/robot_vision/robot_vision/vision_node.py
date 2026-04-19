import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import json
import os
import threading
import time
from collections import Counter

# ======================== 物理层：多线程高效采集 ========================
class CameraStream:
    def __init__(self, dev_id):
        self.cap = cv2.VideoCapture(dev_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.ret, self.frame = self.cap.read()
        self.stopped = False
        self.lock = threading.Lock()

    def start(self):
        threading.Thread(target=self.update, daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            if ret:
                with self.lock: self.frame = frame

    def get_frame(self):
        with self.lock: return self.frame.copy() if self.frame is not None else None

# ======================== 核心逻辑：加权滤波视觉节点 ========================
class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # 1. 核心控制参数
        self.dataset_path = os.path.join(os.getcwd(), 'dataset')
        self.start_time = time.time()
        self.timeout_limit = 5.0      # 5秒强制交卷
        self.gap_threshold = 3.0      # 3倍得分差距判定为“绝对统治”
        self.ultra_conf = 0.985       # 闪电退出的置信度门槛
        
        # 2. 状态存储：单眼记分簿
        self.votes_l = {}  # { (seq_tuple): score }
        self.votes_r = {}
        self.is_published = False

        # 3. 数据集加载
        self.tpl_lib = self.load_dataset()
        
        # 4. 硬件启动
        self.cam_l = CameraStream(0).start()
        self.cam_r = CameraStream(1).start()
        
        # 5. ROS 发布者
        self.pub = self.create_publisher(String, '/vision_detections', 10)
        self.create_timer(0.033, self.vision_engine) # ~30Hz

        self.get_logger().info("视觉终极引擎已启动，正在进行 1.5m 远距离监控...")

    def load_dataset(self):
        lib = {'L': {p: {} for p in range(1, 6)}, 'R': {p: {} for p in range(1, 6)}}
        try:
            for f in os.listdir(self.dataset_path):
                if f.startswith(('L', 'R')) and f.endswith(('.jpg', '.png')):
                    name = os.path.splitext(f)[0]
                    side, label, pos = name[0], int(name[1]), int(name[2])
                    img = cv2.imread(os.path.join(self.dataset_path, f), 0)
                    if img is not None:
                        lib[side][pos][label] = cv2.resize(img, (64, 128))
            self.get_logger().info("50张位置敏感模板加载完毕。")
        except Exception as e:
            self.get_logger().error(f"数据集加载失败: {e}")
        return lib

    # ------------------ 核心处理引擎 ------------------

    def vision_engine(self):
        if self.is_published: return
        elapsed = time.time() - self.start_time

        img_l, img_r = self.cam_l.get_frame(), self.cam_r.get_frame()
        if img_l is None or img_r is None: return

        # 1. 矫正 A4 纸
        warped_l = self.get_a4_warped(img_l)
        warped_r = self.get_a4_warped(img_r)

        # 2. 识别并计分
        res_l = self.match_process(warped_l, 'L')
        res_r = self.match_process(warped_r, 'R')

        # 单眼积分累加 (加权积分制)
        for side, res, votes in [('L', res_l, self.votes_l), ('R', res_r, self.votes_r)]:
            if res:
                seq_tuple = tuple(res['seq'])
                # 使用置信度的 4 次方作为权重，让高分帧具有压倒性优势
                weight = pow(res['conf'], 4)
                votes[seq_tuple] = votes.get(seq_tuple, 0) + weight

        # 3. 检查【闪电退出】条件 (1秒左右快车道)
        best_l = self.get_dominant_winner(self.votes_l)
        best_r = self.get_dominant_winner(self.votes_r)

        if best_l and best_r and best_l == best_r:
            # 只要达成共识且两眼都具有统治地位
            self.publish_result(list(best_l), "Instant_Weighted_Fusion")
            return

        # 4. 检查【5秒超时】强制收敛
        if elapsed >= self.timeout_limit:
            final_seq = self.global_fusion_final()
            self.publish_result(final_seq, "Timeout_Global_Fusion")

    def get_dominant_winner(self, votes_dict):
        """检查单眼内部是否存在绝对领先的冠军"""
        if not votes_dict: return None
        sorted_v = sorted(votes_dict.items(), key=lambda x: x[1], reverse=True)
        
        # 只有一个候选人，且得分累计足够
        if len(sorted_v) == 1:
            return sorted_v[0][0] if sorted_v[0][1] > 1.0 else None
        
        # 检查第一名对第二名的压制倍数
        if sorted_v[0][1] > (sorted_v[1][1] * self.gap_threshold):
            return sorted_v[0][0]
        return None

    def global_fusion_final(self):
        """5秒截止时，合并左右眼所有积分"""
        combined = {}
        all_keys = set(self.votes_l.keys()) | set(self.votes_r.keys())
        for s in all_keys:
            combined[s] = self.votes_l.get(s, 0) + self.votes_r.get(s, 0)
        
        if not combined: return [0,0,0,0,0]
        return list(max(combined, key=combined.get))

    # ------------------ 辅助函数 ------------------

    def match_process(self, warped, side):
        if warped is None: return None
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        step = thresh.shape[1] // 5
        seq, confs = [], []
        for i in range(5):
            pos_id = i + 1
            roi = cv2.resize(thresh[:, i*step : pos_id*step], (64, 128))
            best_s, best_l = -1, None
            for label, tpl in self.tpl_lib[side][pos_id].items():
                res = cv2.matchTemplate(roi, tpl, cv2.TM_CCOEFF_NORMED)
                _, val, _, _ = cv2.minMaxLoc(res)
                if val > best_s: best_s, best_l = val, label
            seq.append(best_l); confs.append(best_s)
        
        if sorted(seq) == [1, 2, 3, 4, 5]:
            return {'seq': seq, 'conf': sum(confs)/5}
        return None

    def get_a4_warped(self, frame):
        gray = cv2.createCLAHE(clipLimit=3.0).apply(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        edged = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 50, 150)
        cnts, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in sorted(cnts, key=cv2.contourArea, reverse=True)[:3]:
            approx = cv2.approxPolyDP(c, 0.02 * cv2.arcLength(c, True), True)
            if len(approx) == 4:
                pts = approx.reshape(4, 2)
                rect = np.zeros((4, 2), dtype="float32")
                s = pts.sum(axis=1)
                rect[0], rect[2] = pts[np.argmin(s)], pts[np.argmax(s)]
                diff = np.diff(pts, axis=1)
                rect[1], rect[3] = pts[np.argmin(diff)], pts[np.argmax(diff)]
                M = cv2.getPerspectiveTransform(rect, np.array([[0,0],[1000,0],[1000,300],[0,300]], dtype="float32"))
                return cv2.warpPerspective(frame, M, (1000, 300))
        return None

    def publish_result(self, seq, mode):
        msg = String()
        msg.data = json.dumps({"sequence": seq, "mode": mode, "time": time.time()-self.start_time})
        self.pub.publish(msg)
        self.is_published = True
        self.get_logger().info(f"任务完成! [{mode}] 结果: {seq}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VisionNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()