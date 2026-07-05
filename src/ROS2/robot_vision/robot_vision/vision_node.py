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
from ament_index_python.packages import get_package_share_directory


# ================= 摄像头线程 =================
class CameraStream:
    def __init__(self, dev_id, width, height, fourcc):
        self.cap = cv2.VideoCapture(dev_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.frame = None
        self.lock = threading.Lock()
        self.stopped = False

    def start(self):
        threading.Thread(target=self.update, daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.frame = frame

    def get_frame(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.stopped = True
        self.cap.release()


# ================= 主节点 =================
class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # ---------- 配置 ----------
        config_dir = os.path.join(get_package_share_directory('robot_vision'), 'config')
        yaml_path = os.path.join(config_dir, 'visison_params.yaml')
        with open(yaml_path, 'r') as f:
            cfg = yaml.safe_load(f)

        self.tw = cfg['warp_target']['width']
        self.th = cfg['warp_target']['height']
        self.match_size = (cfg['match_size']['width'], cfg['match_size']['height'])
        self.stack_size = cfg['frame_stack_size']
        self.full_seq_vote_size = cfg['history_vote_size']
        self.timeout_limit = cfg['timeout_limit']

        self.rois_l = cfg['left_camera']['rois']
        self.rois_r = cfg['right_camera']['rois']
        self.params_l = cfg['left_camera']['preprocessing']
        self.params_r = cfg['right_camera']['preprocessing']

        # ---------- 状态 ----------
        self.buf_l = deque(maxlen=self.stack_size)
        self.buf_r = deque(maxlen=self.stack_size)
        self.global_sequence_history = deque(maxlen=self.full_seq_vote_size)

        self.start_time = time.time()
        self.retry_count = 0
        self.max_retry = 3
        self.is_published = False
        self.final_result_msg = None

        self.declare_parameter('debug_mode', True)
        self.debug_mode = self.get_parameter('debug_mode').value
        self.debug_window_closed = False

        # ---------- 模板 ----------
        self.tpl_lib = self.load_dataset()
        # ❗注意：这里不再强制左右共用模板

        # ---------- 摄像头 ----------
        l = cfg['left_camera']
        r = cfg['right_camera']

        self.cam_l = CameraStream(l['device_id'], l['resolution'][0], l['resolution'][1], l['fourcc']).start()
        self.cam_r = CameraStream(r['device_id'], r['resolution'][0], r['resolution'][1], r['fourcc']).start()

        # ---------- ROS ----------
        self.pub = self.create_publisher(String, '/vision_detections', 10)
        self.create_timer(0.03, self.vision_engine)
        self.create_timer(1.0, self.broadcast_final_result)

        self.get_logger().info("🔥 最终稳定版视觉节点启动（含旋转+Debug）")

    # ================= 数据集 =================
    def load_dataset(self):
        tpls = {'L': {}, 'R': {}}
        pkg_dir = get_package_share_directory('robot_vision')

        for side, folder in [('L', 'dataset_left'), ('R', 'dataset_right')]:
            path = os.path.join(pkg_dir, folder)
            if not os.path.exists(path):
                continue

            for f in os.listdir(path):
                if f.endswith(".png") and "-" in f:
                    label = f.split('-')[1].replace('.png', '')
                    img = cv2.imread(os.path.join(path, f), cv2.IMREAD_GRAYSCALE)
                    if img is not None:
                        img = cv2.resize(img, self.match_size)
                        tpls[side].setdefault(label, []).append(img)
        return tpls

    # ================= 主循环 =================
    def vision_engine(self):
        if self.is_published:
            return

        elapsed = time.time() - self.start_time

        img_l = self.cam_l.get_frame()
        img_r = self.cam_r.get_frame()
        if img_l is None or img_r is None:
            return

        # ✅ 必须保留（你的核心要求）
        img_r = cv2.rotate(img_r, cv2.ROTATE_180)

        dst = np.array([[0,0],[0,self.th],[self.tw,self.th],[self.tw,0]], np.float32)

        w_l = [cv2.warpPerspective(img_l, cv2.getPerspectiveTransform(np.array(p,np.float32), dst),(self.tw,self.th)) for p in self.rois_l]
        w_r = [cv2.warpPerspective(img_r, cv2.getPerspectiveTransform(np.array(p,np.float32), dst),(self.tw,self.th)) for p in self.rois_r]

        self.buf_l.append(cv2.vconcat(w_l).astype(np.float32))
        self.buf_r.append(cv2.vconcat(w_r).astype(np.float32))

        if len(self.buf_l) < self.stack_size:
            return

        bin_l = self.preprocess(self.buf_l, self.params_l)
        bin_r = self.preprocess(self.buf_r, self.params_r)

        res_l = [self.get_digit(bin_l[i*self.th:(i+1)*self.th, 0:self.tw], self.params_l["Center_Dist"], self.tpl_lib['L']) for i in range(4)]
        res_r = [self.get_digit(bin_r[i*self.th:(i+1)*self.th, 0:self.tw], self.params_r["Center_Dist"], self.tpl_lib['R']) for i in range(4)]

        # ---------- 补全 ----------
        full_r = res_r + [self.infer_missing_digit(res_r)]
        full_l = [self.infer_missing_digit(res_l)] + res_l

        # ---------- 融合 ----------
        seq = []
        for i in range(5):
            c = [x for x in [full_l[i], full_r[i]] if x != "N/A"]
            seq.append(Counter(c).most_common(1)[0][0] if c else "N/A")

        # ✅ 强制唯一
        seq = self.enforce_unique(seq)

        # ---------- 合法才入池 ----------
        if len(set(seq)) == 5 and "N/A" not in seq:
            self.global_sequence_history.append(tuple(seq))

        if self.global_sequence_history:
            best = list(Counter(self.global_sequence_history).most_common(1)[0][0])
        else:
            best = ["N/A"]*5

        print(f"\r识别结果: {best}", end="")

        # ---------- Debug ----------
        self.show_debug_image(bin_l, bin_r, res_l, res_r, best, elapsed)

        # ---------- 超时 ----------
        if elapsed >= self.timeout_limit:
            if self.global_sequence_history:
                self.publish_result(best, "success")
            else:
                self.retry()

    # ================= Debug =================
    def show_debug_image(self, bin_l, bin_r, res_l, res_r, seq, elapsed):
        if not self.debug_mode or self.debug_window_closed:
            return

        monitor = cv2.hconcat([bin_r, bin_l])
        monitor = cv2.cvtColor(monitor, cv2.COLOR_GRAY2BGR)

        monitor = cv2.copyMakeBorder(monitor, 80, 0, 0, 0,
                                    cv2.BORDER_CONSTANT, value=(30,30,30))

        cv2.putText(monitor, f"Right: {res_r} | Left: {res_l}",
                    (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0,255,255),2)

        cv2.putText(monitor, f"Fusion: {seq}",
                    (10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0,255,0),2)

        cv2.putText(monitor, f"Time: {elapsed:.2f}s Retry:{self.retry_count}",
                    (10,75), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(255,255,255),2)

        cv2.line(monitor, (self.tw,80),(self.tw, monitor.shape[0]),(0,255,255),1)

        cv2.imshow("Vision_Debug", monitor)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.debug_window_closed = True
            cv2.destroyWindow("Vision_Debug")

    # ================= 工具 =================
    def retry(self):
        self.retry_count += 1
        if self.retry_count >= self.max_retry:
            self.publish_result(["N/A"]*5, "failed")
            return

        self.get_logger().warn(f"重试 {self.retry_count}")
        self.buf_l.clear()
        self.buf_r.clear()
        self.global_sequence_history.clear()
        self.start_time = time.time()

    def enforce_unique(self, seq):
        valid = ['1','2','3','4','5']
        used = set()
        res = ['N/A']*5

        for i,d in enumerate(seq):
            if d in valid and d not in used:
                res[i]=d
                used.add(d)

        remain = [d for d in valid if d not in used]
        for i in range(5):
            if res[i]=="N/A" and remain:
                res[i]=remain.pop(0)
        return res

    def infer_missing_digit(self, seq):
        valid = {'1','2','3','4','5'}
        if any(d not in valid for d in seq):
            return "N/A"
        if len(set(seq))!=4:
            return "N/A"
        return list(valid-set(seq))[0]

    def preprocess(self, buf, p):
        avg = np.mean(buf, axis=0).astype(np.uint8)
        gray = cv2.cvtColor(avg, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, p["Median"]*2+1)
        _,bin_img = cv2.threshold(gray, p["Threshold"],255,cv2.THRESH_BINARY_INV)
        return bin_img

    def get_digit(self, roi, dist_th, tpls):
        cnts,_ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best=None; min_d=1e9

        for c in cnts:
            if cv2.contourArea(c)<50: continue
            x,y,w,h=cv2.boundingRect(c)
            d=abs((x+w//2)-(self.tw//2))
            if d<min_d:
                min_d,best=d,c

        if best is not None and min_d<dist_th:
            x,y,w,h=cv2.boundingRect(best)
            crop=cv2.resize(roi[y:y+h,x:x+w], self.match_size)

            best_s=-1; best_l="N/A"
            for l,t_list in tpls.items():
                for t in t_list:
                    s=cv2.minMaxLoc(cv2.matchTemplate(crop,t,cv2.TM_CCOEFF_NORMED))[1]
                    if s>best_s:
                        best_s,best_l=s,l
            return best_l
        return "N/A"

    def publish_result(self, seq, mode):
        msg = String()
        msg.data = json.dumps({"sequence": seq, "mode": mode})
        self.final_result_msg = msg
        self.pub.publish(msg)
        self.is_published = True
        self.get_logger().info(f"最终结果: {seq}")

    def broadcast_final_result(self):
        if self.final_result_msg is not None:
            self.pub.publish(self.final_result_msg)

    def cleanup(self):
        self.cam_l.stop()
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
