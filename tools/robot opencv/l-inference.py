import cv2
import numpy as np
import json
import os
from collections import deque, Counter

# --- 1. 加载参数与配置 ---
CONFIG_FILE = 'tools/robot opencv/left_camera_params.json'
TEMPLATE_DIR = 'tools/robot opencv/dataset-l'

if not os.path.exists(CONFIG_FILE):
    print(f"错误：未找到配置文件 {CONFIG_FILE}")
    exit()

with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
    config = json.load(f)

CAM_INDEX = 1
ROIS = config['rois']
RES_W, RES_H = config['resolution']
PARAMS = config['preprocessing_params']

TW, TH = 300, 120 
STACK_SIZE = 10     # 图像堆叠数
VOTE_SIZE = 7       # 投票窗口大小（值越大越稳定，但反应稍慢）
MATCH_SIZE = (64, 64)

# 缓冲区初始化
frame_buffer = deque(maxlen=STACK_SIZE)
# 为3个ROI分别建立结果历史队列
vote_history = [deque(maxlen=VOTE_SIZE) for _ in range(3)]

# --- 2. 加载模板库 ---
templates = {}
if os.path.exists(TEMPLATE_DIR):
    for filename in os.listdir(TEMPLATE_DIR):
        if filename.endswith(".png") and "-" in filename:
            digit_label = filename.split('-')[1].replace('.png', '')
            img = cv2.imread(os.path.join(TEMPLATE_DIR, filename), cv2.IMREAD_GRAYSCALE)
            if img is not None:
                img = cv2.resize(img, MATCH_SIZE)
                if digit_label not in templates: templates[digit_label] = []
                templates[digit_label].append(img)
else:
    exit("错误：dataset-l 目录缺失")

# --- 3. 算法逻辑 ---
def filter_by_center_logic(roi_segment, dist_threshold):
    contours, _ = cv2.findContours(roi_segment, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    clean_segment = np.zeros_like(roi_segment)
    roi_center_x = TW // 2 
    best_cnt, min_dist = None, float('inf')

    for cnt in contours:
        if cv2.contourArea(cnt) < 150: continue
        x, y, w, h = cv2.boundingRect(cnt)
        dist = abs((x + w // 2) - roi_center_x)
        if dist < min_dist:
            min_dist, best_cnt = dist, cnt

    if best_cnt is not None and min_dist < dist_threshold:
        cv2.drawContours(clean_segment, [best_cnt], -1, 255, -1)
    return clean_segment, best_cnt

def recognize_digit(clean_seg, cnt):
    if cnt is None: return "N/A", 0.0
    x, y, w, h = cv2.boundingRect(cnt)
    digit_img = cv2.resize(clean_seg[y:y+h, x:x+w], MATCH_SIZE)
    best_score, best_label = -1, "?"
    for label, tpl_list in templates.items():
        for tpl in tpl_list:
            res = cv2.matchTemplate(digit_img, tpl, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, _ = cv2.minMaxLoc(res)
            if max_val > best_score:
                best_score, best_label = max_val, label
    return best_label, best_score

# --- 4. 主程序 ---
cap = cv2.VideoCapture(CAM_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, RES_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RES_H)

print("\n>>> 已启用【投票法滤波】识别模式")

while True:
    ret, frame = cap.read()
    # frame = cv2.rotate(frame, cv2.ROTATE_180)
    if not ret: break

    current_warps = [cv2.warpPerspective(frame, cv2.getPerspectiveTransform(np.array(pts, np.float32), 
                     np.array([[0,0],[0,TH],[TW,TH],[TW,0]], np.float32)), (TW, TH)) for pts in ROIS]
    frame_buffer.append(cv2.vconcat(current_warps).astype(np.float32))

    if len(frame_buffer) < STACK_SIZE: continue

    # 预处理
    avg_stack = np.mean(frame_buffer, axis=0).astype(np.uint8)
    gray = cv2.cvtColor(avg_stack, cv2.COLOR_BGR2GRAY)
    gray = cv2.createCLAHE(clipLimit=PARAMS["CLAHE"], tileGridSize=(8,8)).apply(gray)
    gray = cv2.medianBlur(gray, PARAMS["Median"] * 2 + 1)
    _, binary = cv2.threshold(gray, PARAMS["Threshold"], 255, cv2.THRESH_BINARY_INV)
    if PARAMS["Morph_Size"] > 0:
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, np.ones((PARAMS["Morph_Size"], PARAMS["Morph_Size"]), np.uint8))

    display_frame = frame.copy()
    current_frame_results = [] # 存储本轮投票后的结果

    for i in range(3):
        seg = binary[i*TH : (i+1)*TH, 0:TW]
        clean_seg, best_cnt = filter_by_center_logic(seg, PARAMS["Center_Dist"])
        
        # 1. 获得原始识别结果
        raw_label, score = recognize_digit(clean_seg, best_cnt)
        
        # 2. 将结果存入对应的投票历史队列
        vote_history[i].append(raw_label)
        
        # 3. 投票逻辑：获取队列中出现次数最多的结果
        votes = Counter(vote_history[i])
        voted_label = votes.most_common(1)[0][0]
        current_frame_results.append(voted_label)
        
        # 绘制
        color = (0, 255, 0) if score > 0.8 else (0, 0, 255)
        cv2.polylines(display_frame, [np.array(ROIS[i], np.int32)], True, color, 2)
        # 显示投票后的结果，括号内显示原始置信度
        cv2.putText(display_frame, f"{voted_label} (S:{score:.2f})", tuple(ROIS[i][0]), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    # 终端输出当前稳定的识别序列
    print(f"稳定输出流: {current_frame_results}", end='\r')

    cv2.imshow("Voted_Left_Recognition", display_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()