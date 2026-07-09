import cv2
import numpy as np
import json
import os
from collections import deque, Counter

# --- 1. 初始化与配置加载 ---
def load_cam_config(file_path):
    if not os.path.exists(file_path):
        exit(f"错误：未找到配置文件 {file_path}")
    with open(file_path, 'r', encoding='utf-8') as f:
        return json.load(f)

left_cfg = load_cam_config('left_camera_params.json')   #
right_cfg = load_cam_config('right_camera_params.json') #

# 提取参数
CAM_L_IDX, CAM_R_IDX = 1, 2
ROIS_L, PARAMS_L = left_cfg['rois'], left_cfg['preprocessing_params']   #
ROIS_R, PARAMS_R = right_cfg['rois'], right_cfg['preprocessing_params'] #

# 全局算法常量
TW, TH = 300, 120
STACK_SIZE = 10
VOTE_SIZE = 15  # 全局投票窗口大小，增加到15以获得更高稳定性
MATCH_SIZE = (64, 64)

# 缓冲区初始化
buf_l = deque(maxlen=STACK_SIZE)
buf_r = deque(maxlen=STACK_SIZE)
# 最终五个位置的投票历史队列
# Pos 0: R[0], Pos 1: R[1]&L[0], Pos 2: R[2]&L[1], Pos 3: R[3]&L[2], Pos 4: L[3]
final_vote_history = [deque(maxlen=VOTE_SIZE) for _ in range(5)]

# --- 2. 模板加载函数 ---
def load_templates(path):
    tpls = {}
    if os.path.exists(path):
        for f in os.listdir(path):
            if f.endswith(".png") and "-" in f:
                label = f.split('-')[1].replace('.png', '')
                img = cv2.imread(os.path.join(path, f), cv2.IMREAD_GRAYSCALE)
                if img is not None:
                    img = cv2.resize(img, MATCH_SIZE)
                    if label not in tpls: tpls[label] = []
                    tpls[label].append(img)
    return tpls

templates_l = load_templates('dataset-l') #
templates_r = load_templates('dataset-r') #

# --- 3. 核心处理函数 ---
def preprocess_frame(frame_buf, params, side='l'):
    avg = np.mean(frame_buf, axis=0).astype(np.uint8)
    gray = cv2.cvtColor(avg, cv2.COLOR_BGR2GRAY)
    if params["CLAHE"] > 0:
        gray = cv2.createCLAHE(clipLimit=params["CLAHE"], tileGridSize=(8,8)).apply(gray)
    gray = cv2.medianBlur(gray, params["Median"] * 2 + 1)
    _, binary = cv2.threshold(gray, params["Threshold"], 255, cv2.THRESH_BINARY_INV)
    if params["Morph_Size"] > 0:
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, np.ones((params["Morph_Size"], params["Morph_Size"]), np.uint8))
    return binary

def get_digit(roi_seg, dist_threshold, tpls):
    contours, _ = cv2.findContours(roi_seg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    roi_center_x = TW // 2
    best_cnt, min_dist = None, float('inf')
    for cnt in contours:
        if cv2.contourArea(cnt) < 50: continue # 统一使用较小面积以适应远景
        dist = abs((cv2.boundingRect(cnt)[0] + cv2.boundingRect(cnt)[2]//2) - roi_center_x)
        if dist < min_dist: min_dist, best_cnt = dist, cnt
    
    if best_cnt is not None and min_dist < dist_threshold:
        x, y, w, h = cv2.boundingRect(best_cnt)
        digit_img = cv2.resize(roi_seg[y:y+h, x:x+w], MATCH_SIZE)
        best_score, best_label = -1, "N/A"
        for label, tpl_list in tpls.items():
            for tpl in tpl_list:
                res = cv2.matchTemplate(digit_img, tpl, cv2.TM_CCOEFF_NORMED)
                score = cv2.minMaxLoc(res)[1]
                if score > best_score: best_score, best_label = score, label
        return best_label
    return "N/A"

# --- 4. 主循环 ---
cap_l = cv2.VideoCapture(CAM_L_IDX)
cap_r = cv2.VideoCapture(CAM_R_IDX)
for cap in [cap_l, cap_r]:
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

print("\n>>> 双路摄像头融合识别系统已启动...")

while True:
    ret_l, frame_l = cap_l.read()
    ret_r, frame_r = cap_r.read()
    if not ret_l or not ret_r: break

    # A. 矫正与堆叠
    warps_l = [cv2.warpPerspective(frame_l, cv2.getPerspectiveTransform(np.array(p, np.float32), 
               np.array([[0,0],[0,TH],[TW,TH],[TW,0]], np.float32)), (TW, TH)) for p in ROIS_L]
    warps_r = [cv2.warpPerspective(frame_r, cv2.getPerspectiveTransform(np.array(p, np.float32), 
               np.array([[0,0],[0,TH],[TW,TH],[TW,0]], np.float32)), (TW, TH)) for p in ROIS_R]
    
    buf_l.append(cv2.vconcat(warps_l).astype(np.float32))
    buf_r.append(cv2.vconcat(warps_r).astype(np.float32))

    if len(buf_l) < STACK_SIZE: continue

    # B. 预处理
    bin_l = preprocess_frame(buf_l, PARAMS_L)
    bin_r = preprocess_frame(buf_r, PARAMS_R)

    # C. 提取各位置识别结果 (N/A, 1, 2, 3...)
    res_r = [get_digit(bin_r[i*TH:(i+1)*TH, 0:TW], PARAMS_R["Center_Dist"], templates_r) for i in range(4)]
    res_l = [get_digit(bin_l[i*TH:(i+1)*TH, 0:TW], PARAMS_L["Center_Dist"], templates_l) for i in range(4)]

    # D. 综合并注入投票历史
    # 位置1：仅右眼
    if res_r[0] != "N/A": final_vote_history[0].append(res_r[0])
    # 位置2,3,4：双眼重叠观察
    for i in range(3):
        if res_r[i+1] != "N/A": final_vote_history[i+1].append(res_r[i+1])
        if res_l[i] != "N/A": final_vote_history[i+1].append(res_l[i])
    # 位置5：仅左眼
    if res_l[3] != "N/A": final_vote_history[4].append(res_l[3])

    # E. 最终投票输出
    final_sequence = []
    for hist in final_vote_history:
        if hist:
            final_sequence.append(Counter(hist).most_common(1)[0][0])
        else:
            final_sequence.append("?")

    print(f"最终融合序列: {final_sequence}", end='\r')

    # 显示二值化参考
    cv2.imshow("Combined_Binary_View", cv2.hconcat([bin_r, bin_l]))
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap_l.release()
cap_r.release()
cv2.destroyAllWindows()