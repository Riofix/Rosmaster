import cv2
import numpy as np
import json
import os
from collections import deque, Counter

# --- 1. 配置加载 ---
def load_cfg(path):
    with open(path, 'r', encoding='utf-8') as f: return json.load(f)

cfg_l = load_cfg('left_camera_params.json')   #
cfg_r = load_cfg('right_camera_params.json') #

ROIS_L, PARAMS_L = cfg_l['rois'], cfg_l['preprocessing_params'] #
ROIS_R, PARAMS_R = cfg_r['rois'], cfg_r['preprocessing_params'] #

CAM_L_IDX, CAM_R_IDX = 1, 2
TW, TH = 300, 120
STACK_SIZE = 10
FULL_SEQ_VOTE_SIZE = 10 # 针对最终5位序列的投票深度
MATCH_SIZE = (64, 64)

# 缓冲区
buf_l, buf_r = deque(maxlen=STACK_SIZE), deque(maxlen=STACK_SIZE)
# 终极保险：存储合法的 5 位全排列序列历史
global_sequence_history = deque(maxlen=FULL_SEQ_VOTE_SIZE)
last_stable_sequence = ["?", "?", "?", "?", "?"]

# --- 2. 模板加载 ---
def load_tpls(path):
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

templates_l = load_tpls('dataset-l') #
templates_r = load_tpls('dataset-r') #

# --- 3. 核心算法 ---
def preprocess(buf, p):
    avg = np.mean(buf, axis=0).astype(np.uint8)
    gray = cv2.cvtColor(avg, cv2.COLOR_BGR2GRAY)
    if p["CLAHE"] > 0: gray = cv2.createCLAHE(clipLimit=p["CLAHE"], tileGridSize=(8,8)).apply(gray)
    gray = cv2.medianBlur(gray, p["Median"] * 2 + 1)
    _, bin_img = cv2.threshold(gray, p["Threshold"], 255, cv2.THRESH_BINARY_INV)
    if p["Morph_Size"] > 0:
        bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, np.ones((p["Morph_Size"], p["Morph_Size"]), np.uint8))
    return bin_img

def get_digit(roi_seg, dist_th, tpls):
    cnts, _ = cv2.findContours(roi_seg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_cnt, min_d = None, float('inf')
    for c in cnts:
        if cv2.contourArea(c) < 50: continue
        d = abs((cv2.boundingRect(c)[0] + cv2.boundingRect(c)[2]//2) - (TW//2))
        if d < min_d: min_d, best_cnt = d, c
    if best_cnt is not None and min_d < dist_th:
        x, y, w, h = cv2.boundingRect(best_cnt)
        crop = cv2.resize(roi_seg[y:y+h, x:x+w], MATCH_SIZE)
        best_s, best_l = -1, "N/A"
        for l, t_list in tpls.items():
            for t in t_list:
                s = cv2.minMaxLoc(cv2.matchTemplate(crop, t, cv2.TM_CCOEFF_NORMED))[1]
                if s > best_s: best_s, best_l = s, l
        return best_l
    return "N/A"

def infer_missing_digit(seq_4):
    """逻辑补全：如果4个数字不重复且都在1-5内，推知第5个数字"""
    valid_digits = {'1', '2', '3', '4', '5'}
    found = {d for d in seq_4 if d in valid_digits}
    if len(found) == 4:
        missing = list(valid_digits - found)[0]
        return missing
    return "N/A"

# --- 4. 主循环 ---
cap_l, cap_r = cv2.VideoCapture(CAM_L_IDX), cv2.VideoCapture(CAM_R_IDX)
for c in [cap_l, cap_r]:
    c.set(cv2.CAP_PROP_FRAME_WIDTH, 1280); c.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while True:
    ret_l, frame_l = cap_l.read(); ret_r, frame_r = cap_r.read()
    if not ret_l or not ret_r: break
    frame_r = cv2.rotate(frame_r, cv2.ROTATE_180)

    # A. 矫正堆叠
    w_l = [cv2.warpPerspective(frame_l, cv2.getPerspectiveTransform(np.array(p, np.float32), 
           np.array([[0,0],[0,TH],[TW,TH],[TW,0]], np.float32)), (TW, TH)) for p in ROIS_L]
    w_r = [cv2.warpPerspective(frame_r, cv2.getPerspectiveTransform(np.array(p, np.float32), 
           np.array([[0,0],[0,TH],[TW,TH],[TW,0]], np.float32)), (TW, TH)) for p in ROIS_R]
    buf_l.append(cv2.vconcat(w_l).astype(np.float32)); buf_r.append(cv2.vconcat(w_r).astype(np.float32))

    if len(buf_l) < STACK_SIZE: continue

    # B. 识别原始结果
    bin_l, bin_r = preprocess(buf_l, PARAMS_L), preprocess(buf_r, PARAMS_R)
    res_l = [get_digit(bin_l[i*TH:(i+1)*TH, 0:TW], PARAMS_L["Center_Dist"], templates_l) for i in range(4)]
    res_r = [get_digit(bin_r[i*TH:(i+1)*TH, 0:TW], PARAMS_R["Center_Dist"], templates_r) for i in range(4)]

    # C. 逻辑补全与融合 (利用 12345 唯一性)
    # 右眼推知 Pos 4, 左眼推知 Pos 0
    inferred_pos4 = infer_missing_digit(res_r)
    inferred_pos0 = infer_missing_digit(res_l)

    full_r = res_r + [inferred_pos4]      # 右眼推测全序列 [R0, R1, R2, R3, R4_inf]
    full_l = [inferred_pos0] + res_l      # 左眼推测全序列 [L0_inf, L1, L2, L3, L4]

    # D. 全局全排列校验
    current_best_seq = []
    for p in range(5):
        # 对每个位置，结合双眼推测结果进行局部表决
        candidates = [c for c in [full_r[p], full_l[p]] if c != "N/A"]
        current_best_seq.append(Counter(candidates).most_common(1)[0][0] if candidates else "N/A")

    # 【最后一道保险】：检查是否为 1-5 的合法排列
    if len(set(current_best_seq)) == 5 and "N/A" not in current_best_seq:
        # 只有合法的全排列序列才进入最终投票池
        global_sequence_history.append(tuple(current_best_seq))

    # E. 最终投票输出
    if global_sequence_history:
        stable_seq = list(Counter(global_sequence_history).most_common(1)[0][0])
        last_stable_sequence = stable_seq

    print(f"最终保险校验序列: {last_stable_sequence}", end='\r')

    cv2.imshow("Binary_Monitor", cv2.hconcat([bin_r, bin_l]))
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap_l.release(); cap_r.release(); cv2.destroyAllWindows()