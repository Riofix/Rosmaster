import cv2
import numpy as np
import json
import os
from collections import deque, Counter

# --- 1. 加载右摄像头参数 ---
CONFIG_FILE = 'tools/robot opencv/right_camera_params.json'
TEMPLATE_DIR = 'tools/robot opencv/dataset-r'  

if not os.path.exists(CONFIG_FILE):
    print(f"错误：未找到配置文件 {CONFIG_FILE}")
    exit()

with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
    config = json.load(f)

CAM_INDEX = 2  # 右摄像头
ROIS = config['rois']
RES_W, RES_H = config['resolution']
PARAMS = config['preprocessing_params']

TW, TH = 300, 120 
STACK_SIZE = 10     # 图像平均堆叠帧数
VOTE_SIZE = 7       # 投票窗口大小（建议 7-10）
MATCH_SIZE = (64, 64)

# 缓冲区初始化
frame_buffer = deque(maxlen=STACK_SIZE)
# 为右侧3个ROI分别建立结果历史队列
vote_history = [deque(maxlen=VOTE_SIZE) for _ in range(3)]

# --- 2. 加载模板库 ---
templates = {}
if os.path.exists(TEMPLATE_DIR):
    print(f"正在从 {TEMPLATE_DIR} 加载右摄像头模板...")
    for filename in os.listdir(TEMPLATE_DIR):
        if filename.endswith(".png") and "-" in filename:
            try:
                digit_label = filename.split('-')[1].replace('.png', '')
                img = cv2.imread(os.path.join(TEMPLATE_DIR, filename), cv2.IMREAD_GRAYSCALE)
                if img is not None:
                    img = cv2.resize(img, MATCH_SIZE)
                    if digit_label not in templates: templates[digit_label] = []
                    templates[digit_label].append(img)
            except: continue
    print(f"模板加载完成，类别: {list(templates.keys())}")
else:
    print(f"警告：未找到 {TEMPLATE_DIR}")

# --- 3. 核心算法逻辑 ---
def filter_by_center_logic(roi_segment, dist_threshold):
    contours, _ = cv2.findContours(roi_segment, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    clean_segment = np.zeros_like(roi_segment)
    roi_center_x = TW // 2 
    best_cnt, min_dist = None, float('inf')

    for cnt in contours:
        # 右侧远景，面积阈值保持为 50
        if cv2.contourArea(cnt) < 50: continue 
        x, y, w, h = cv2.boundingRect(cnt)
        dist = abs((x + w // 2) - roi_center_x)
        if dist < min_dist:
            min_dist, best_cnt = dist, cnt

    if best_cnt is not None and min_dist < dist_threshold:
        cv2.drawContours(clean_segment, [best_cnt], -1, 255, -1)
    return clean_segment, best_cnt

def recognize_digit(clean_seg, cnt):
    if cnt is None or len(templates) == 0: return "N/A", 0.0
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

print("\n>>> 右摄像头识别模式启动（已启用投票滤波）")

while True:
    ret, frame = cap.read()
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    if not ret: break

    current_warps = [cv2.warpPerspective(frame, cv2.getPerspectiveTransform(np.array(pts, np.float32), 
                     np.array([[0,0],[0,TH],[TW,TH],[TW,0]], np.float32)), (TW, TH)) for pts in ROIS]
    frame_buffer.append(cv2.vconcat(current_warps).astype(np.float32))

    if len(frame_buffer) < STACK_SIZE: continue

    avg_stack = np.mean(frame_buffer, axis=0).astype(np.uint8)
    gray = cv2.cvtColor(avg_stack, cv2.COLOR_BGR2GRAY)
    
    # 按照右侧锁定参数处理
    if PARAMS["CLAHE"] > 0:
        gray = cv2.createCLAHE(clipLimit=PARAMS["CLAHE"], tileGridSize=(8,8)).apply(gray)
    
    gray = cv2.medianBlur(gray, PARAMS["Median"] * 2 + 1)
    _, binary = cv2.threshold(gray, PARAMS["Threshold"], 255, cv2.THRESH_BINARY_INV)
    
    if PARAMS["Morph_Size"] > 0:
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, np.ones((PARAMS["Morph_Size"], PARAMS["Morph_Size"]), np.uint8))

    display_frame = frame.copy()
    final_clean_list = []
    voted_results = [] # 存储四个盒子投票后的最终结果
    
    for i in range(3):
        seg = binary[i*TH : (i+1)*TH, 0:TW]
        clean_seg, best_cnt = filter_by_center_logic(seg, PARAMS["Center_Dist"])
        final_clean_list.append(clean_seg)
        
        # 1. 获取当前帧识别结果
        raw_label, score = recognize_digit(clean_seg, best_cnt)
        
        # 2. 存入投票历史队列
        vote_history[i].append(raw_label)
        
        # 3. 统计投票：取过去 VOTE_SIZE 次里出现最多的
        counter = Counter(vote_history[i])
        voted_label = counter.most_common(1)[0][0]
        voted_results.append(voted_label)
        
        # 绘图显示
        color = (0, 255, 0) if score > 0.75 else (0, 0, 255)
        cv2.polylines(display_frame, [np.array(ROIS[i], np.int32)], True, color, 2)
        # 显示投票结果 [V]，同时保留当前置信度 (S) 供参考
        cv2.putText(display_frame, f"R:{voted_label} (S:{score:.2f})", tuple(ROIS[i][0]), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    # 终端实时输出
    print(f"右侧稳定序列: {voted_results}", end='\r')

    cv2.imshow("Right_Voted_Recognition", display_frame)
    cv2.imshow("Right_Binary_Filtered", cv2.vconcat(final_clean_list))

    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()