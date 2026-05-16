import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import json
import os
import threading
import time
from collections import deque, Counter
import yaml

# ======================== 物理层：多线程高效采集 ========================
class CameraStream:
    def __init__(self, dev_id, width, height, fourcc):
        self.cap = cv2.VideoCapture(dev_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
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

    def stop(self):
        self.stopped = True
        self.cap.release()

# ======================== 核心逻辑：固定ROI + 帧堆叠 + 逻辑补全 ========================
class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # ========== 1. 从 YAML 加载所有参数 (唯一配置来源) ==========
        config_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
        yaml_path = os.path.join(config_dir, 'visison_params.yaml')
        with open(yaml_path, 'r') as f:
            cfg = yaml.safe_load(f)

        # ----- 管线参数 -----
        self.tw = cfg['warp_target']['width']
        self.th = cfg['warp_target']['height']
        self.match_size = (cfg['match_size']['width'], cfg['match_size']['height'])
        self.stack_size = cfg['frame_stack_size']
        self.full_seq_vote_size = cfg['history_vote_size']
        self.timeout_limit = cfg['timeout_limit']
        self.debug_mode = cfg.get('debug_mode', False)   # 调试模式开关

        # ----- 左相机参数 -----
        lcam = cfg['left_camera']
        self.rois_l = lcam['rois']
        self.params_l = lcam['preprocessing']
        left_dev = lcam['device_id']
        left_res = lcam['resolution']
        left_fourcc = lcam['fourcc']

        # ----- 右相机参数 -----
        rcam = cfg['right_camera']
        self.rois_r = rcam['rois']
        self.params_r = rcam['preprocessing']
        right_dev = rcam['device_id']
        right_res = rcam['resolution']
        right_fourcc = rcam['fourcc']

        # ========== 2. 状态变量 ==========
        self.start_time = time.time()
        self.is_published = False
        self.debug_window_closed = False
        self.buf_l = deque(maxlen=self.stack_size)
        self.buf_r = deque(maxlen=self.stack_size)
        self.global_sequence_history = deque(maxlen=self.full_seq_vote_size)

        # ========== 3. 数据集加载 ==========
        self.tpl_lib = self.load_dataset()

        # ========== 4. 硬件启动 (参数来自 YAML) ==========
        self.cam_l = CameraStream(left_dev, left_res[0], left_res[1], left_fourcc).start()
        self.cam_r = CameraStream(right_dev, right_res[0], right_res[1], right_fourcc).start()

        # ========== 5. ROS 发布者 ==========
        self.pub = self.create_publisher(String, '/vision_detections', 10)
        self.create_timer(0.033, self.vision_engine)  # ~30Hz

        self.get_logger().info("视觉引擎已启动 (固定ROI + 帧堆叠 + 逻辑补全)...")

    # ------------------ 数据集加载 ------------------

    def load_dataset(self):
        """从 dataset_left/ 和 dataset_right/ 加载模板，按数字标签分组"""
        tpls = {'L': {}, 'R': {}}
        paths = {
            'L': os.path.join(os.getcwd(), 'dataset_left'),
            'R': os.path.join(os.getcwd(), 'dataset_right')
        }
        for side, path in paths.items():
            if os.path.exists(path):
                for f in os.listdir(path):
                    if f.endswith(".png") and "-" in f:
                        label = f.split('-')[1].replace('.png', '')
                        img = cv2.imread(os.path.join(path, f), cv2.IMREAD_GRAYSCALE)
                        if img is not None:
                            img = cv2.resize(img, self.match_size)
                            if label not in tpls[side]:
                                tpls[side][label] = []
                            tpls[side][label].append(img)
                self.get_logger().info(f"{side}眼模板加载完成，共{sum(len(v) for v in tpls[side].values())}张")
            else:
                self.get_logger().warn(f"{side}眼数据集路径不存在: {path}")
        return tpls

    # ------------------ 核心处理引擎 ------------------

    def vision_engine(self):
        """主循环：固定ROI矫正 → 帧堆叠 → 预处理 → 轮廓筛选 → 模板匹配 → 逻辑补全 → 历史投票"""
        if self.is_published: return
        elapsed = time.time() - self.start_time

        img_l, img_r = self.cam_l.get_frame(), self.cam_r.get_frame()
        if img_l is None or img_r is None: return

        # 1. 固定ROI透视变换 — 将每个 ROI 矫正为 tw×th 的矩形
        dst_pts = np.array([[0, 0], [0, self.th], [self.tw, self.th], [self.tw, 0]], dtype=np.float32)
        w_l = [cv2.warpPerspective(img_l,
               cv2.getPerspectiveTransform(np.array(p, np.float32), dst_pts),
               (self.tw, self.th)) for p in self.rois_l]
        w_r = [cv2.warpPerspective(img_r,
               cv2.getPerspectiveTransform(np.array(p, np.float32), dst_pts),
               (self.tw, self.th)) for p in self.rois_r]

        # 2. 垂直堆叠 4 个区域 → 入帧缓冲区
        self.buf_l.append(cv2.vconcat(w_l).astype(np.float32))
        self.buf_r.append(cv2.vconcat(w_r).astype(np.float32))

        # 缓冲区未满时等待
        if len(self.buf_l) < self.stack_size: return

        # 3. 预处理 (帧平均 → CLAHE → 中值滤波 → 二值化 → 形态学)
        bin_l = self.preprocess(self.buf_l, self.params_l)
        bin_r = self.preprocess(self.buf_r, self.params_r)

        # 4. 在 4 个区域中分别识别数字 (轮廓中心筛选 + 模板匹配)
        res_l = [self.get_digit(bin_l[i*self.th:(i+1)*self.th, 0:self.tw],
                  self.params_l["Center_Dist"], self.tpl_lib['L']) for i in range(4)]
        res_r = [self.get_digit(bin_r[i*self.th:(i+1)*self.th, 0:self.tw],
                  self.params_r["Center_Dist"], self.tpl_lib['R']) for i in range(4)]

        # 5. 逻辑补全 — 利用 1-5 不重复特性推断缺失的数字
        #    右眼能看到位置 0-3，推知位置 4
        #    左眼能看到位置 1-4，推知位置 0
        inferred_pos4 = self.infer_missing_digit(res_r)   # 右眼推位置4
        inferred_pos0 = self.infer_missing_digit(res_l)   # 左眼推位置0
        full_r = res_r + [inferred_pos4]                  # [R0, R1, R2, R3, R4_inf]
        full_l = [inferred_pos0] + res_l                  # [L0_inf, L1, L2, L3, L4]

        # 6. 双眼融合 — 每个位置结合左右眼结果进行局部表决
        current_best_seq = []
        for p in range(5):
            candidates = [c for c in [full_r[p], full_l[p]] if c != "N/A"]
            current_best_seq.append(Counter(candidates).most_common(1)[0][0] if candidates else "N/A")

        # 7. 全排列校验 — 只有合法的 1-5 不重复序列才进入投票池
        if len(set(current_best_seq)) == 5 and "N/A" not in current_best_seq:
            self.global_sequence_history.append(tuple(current_best_seq))

        # 8. 调试模式：每帧结果只打印日志，不发布话题
        if self.debug_mode:
            # 日志输出当前识别的左右眼原始结果
            debug_msg = (f"[DEBUG] 左眼(R0-R3): {res_l} | 右眼(L1-L4): {res_r}"
                         f" | 补全后: {current_best_seq}")
            self.get_logger().info(debug_msg)
            self.show_debug_image(bin_l, bin_r, res_l, res_r, current_best_seq, elapsed)

            # 超时后才发布最终结果到话题
            if elapsed >= self.timeout_limit:
                if self.global_sequence_history:
                    final_seq = list(Counter(self.global_sequence_history).most_common(1)[0][0])
                    self.publish_result(final_seq, "Timeout_Historical_Fusion")
                else:
                    self.publish_result(['N/A'] * 5, "Timeout_NoResult")
        else:
            # 正常模式：一旦历史有稳定序列，立即发布
            if elapsed >= self.timeout_limit:
                if self.global_sequence_history:
                    final_seq = list(Counter(self.global_sequence_history).most_common(1)[0][0])
                    self.publish_result(final_seq, "Timeout_Historical_Fusion")
                else:
                    self.publish_result(['N/A'] * 5, "Timeout_NoResult")
            elif self.global_sequence_history:
                final_seq = list(Counter(self.global_sequence_history).most_common(1)[0][0])
                self.publish_result(final_seq, "Early_Historical_Fusion")

    # ------------------ 帧堆叠预处理 ------------------

    # ------------------ Debug display ------------------

    def show_debug_image(self, bin_l, bin_r, res_l, res_r, current_best_seq, elapsed):
        if self.debug_window_closed:
            return

        monitor = cv2.hconcat([bin_r, bin_l])
        monitor = cv2.cvtColor(monitor, cv2.COLOR_GRAY2BGR)
        monitor = cv2.copyMakeBorder(monitor, 70, 0, 0, 0, cv2.BORDER_CONSTANT, value=(30, 30, 30))

        cv2.putText(monitor, f"Right: {res_r}  Left: {res_l}", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(monitor, f"Best: {current_best_seq}  Time: {elapsed:.2f}s  Press q to close",
                    (10, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.line(monitor, (self.tw, 70), (self.tw, monitor.shape[0]), (0, 255, 255), 1)

        cv2.imshow("Vision_Debug_Binary", monitor)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.debug_window_closed = True
            cv2.destroyWindow("Vision_Debug_Binary")

    def preprocess(self, buf, params):
        """
        帧堆叠 + 预处理管线:
        1. 多帧平均去噪
        2. CLAHE 自适应直方图均衡
        3. 中值滤波
        4. 二值化 (THRESH_BINARY_INV)
        5. 形态学开运算去噪点
        """
        avg = np.mean(buf, axis=0).astype(np.uint8)
        gray = cv2.cvtColor(avg, cv2.COLOR_BGR2GRAY)
        if params["CLAHE"] > 0:
            gray = cv2.createCLAHE(clipLimit=params["CLAHE"], tileGridSize=(8, 8)).apply(gray)
        gray = cv2.medianBlur(gray, params["Median"] * 2 + 1)
        _, bin_img = cv2.threshold(gray, params["Threshold"], 255, cv2.THRESH_BINARY_INV)
        if params["Morph_Size"] > 0:
            bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN,
                                       np.ones((params["Morph_Size"], params["Morph_Size"]), np.uint8))
        return bin_img

    # ------------------ 轮廓筛选 + 模板匹配 ------------------

    def get_digit(self, roi_seg, dist_th, tpls):
        """
        在二值化 ROI 中:
        1. 查找所有轮廓，过滤面积 < 50 的噪点
        2. 按轮廓中心到 ROI 中心的水平距离排序，选最近且 < dist_th 的
        3. 裁剪数字区域 → 缩放到 match_size → TM_CCOEFF_NORMED 匹配
        """
        cnts, _ = cv2.findContours(roi_seg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_cnt, min_d = None, float('inf')
        for c in cnts:
            if cv2.contourArea(c) < 50: continue
            d = abs((cv2.boundingRect(c)[0] + cv2.boundingRect(c)[2] // 2) - (self.tw // 2))
            if d < min_d:
                min_d, best_cnt = d, c
        if best_cnt is not None and min_d < dist_th:
            x, y, w, h = cv2.boundingRect(best_cnt)
            crop = cv2.resize(roi_seg[y:y + h, x:x + w], self.match_size)
            best_s, best_l = -1, "N/A"
            for l, t_list in tpls.items():
                for t in t_list:
                    s = cv2.minMaxLoc(cv2.matchTemplate(crop, t, cv2.TM_CCOEFF_NORMED))[1]
                    if s > best_s:
                        best_s, best_l = s, l
            return best_l
        return "N/A"

    # ------------------ 逻辑补全 ------------------

    def infer_missing_digit(self, seq_4):
        """
        利用 1-5 不重复特性:
        如果4个数字互不相同且都在1-5范围内，推知缺失的第5个数字
        例如: [1,2,3,4] → 5,  [2,3,4,5] → 1
        """
        valid_digits = {'1', '2', '3', '4', '5'}
        found = {d for d in seq_4 if d in valid_digits}
        if len(found) == 4:
            return list(valid_digits - found)[0]
        return "N/A"

    # ------------------ 发布结果 ------------------

    def publish_result(self, seq, mode):
        msg = String()
        msg.data = json.dumps({"sequence": seq, "mode": mode, "time": time.time() - self.start_time})
        self.pub.publish(msg)
        self.is_published = True
        self.get_logger().info(f"任务完成! [{mode}] 结果: {seq}")

    def cleanup(self):
        if hasattr(self, 'cam_l'):
            self.cam_l.stop()
        if hasattr(self, 'cam_r'):
            self.cam_r.stop()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
