import cv2
import numpy as np
import json
from collections import deque

# --- 1. 初始化配置 ---
try:
    with open('l-roi_config.json', 'r') as f:
        config = json.load(f)
    rois = config['rois']
    W_RES, H_RES = config['resolution']
except Exception as e:
    print(f"配置文件读取失败: {e}")
    exit()

TW, TH = 300, 120 
STACK_SIZE = 10
frame_buffer = deque(maxlen=STACK_SIZE)

def nothing(x):
    pass

# --- 2. 辅助函数：中心判定过滤 ---
def filter_by_center_logic(roi_segment, dist_threshold=80):
    """只保留距离水平中心线最近的轮廓"""
    contours, _ = cv2.findContours(roi_segment, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    clean_segment = np.zeros_like(roi_segment)
    roi_center_x = TW // 2 
    
    best_cnt = None
    min_dist_to_center = float('inf')

    for cnt in contours:
        if cv2.contourArea(cnt) < 150: # 过滤极小噪点
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        cnt_center_x = x + (w // 2)
        dist = abs(cnt_center_x - roi_center_x)
        
        if dist < min_dist_to_center:
            min_dist_to_center = dist
            best_cnt = cnt

    if best_cnt is not None and min_dist_to_center < dist_threshold:
        cv2.drawContours(clean_segment, [best_cnt], -1, 255, -1)
    return clean_segment

# --- 3. 创建调参面板 ---
cv2.namedWindow("Tuning_Panel", cv2.WINDOW_NORMAL)
cv2.createTrackbar("CLAHE", "Tuning_Panel", 2, 10, nothing)
cv2.createTrackbar("Median", "Tuning_Panel", 1, 5, nothing)
cv2.createTrackbar("Threshold", "Tuning_Panel", 127, 255, nothing)
cv2.createTrackbar("Morph_Size", "Tuning_Panel", 2, 10, nothing)
cv2.createTrackbar("Center_Dist", "Tuning_Panel", 80, 150, nothing) # 实时调节中心判定范围

# --- 4. 启动摄像头 ---
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, W_RES)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H_RES)

print("系统启动成功：正在进行10帧堆叠预热...")

while True:
    ret, frame = cap.read()
    if not ret: break

    # A. 执行透视变换并堆叠
    current_warps = []
    for pts in rois:
        src = np.array(pts, dtype=np.float32)
        dst = np.array([[0,0],[0,TH],[TW,TH],[TW,0]], dtype=np.float32)
        M = cv2.getPerspectiveTransform(src, dst)
        current_warps.append(cv2.warpPerspective(frame, M, (TW, TH)))
    
    frame_buffer.append(cv2.vconcat(current_warps).astype(np.float32))

    if len(frame_buffer) < STACK_SIZE:
        continue

    # B. 获取10帧平均图
    avg_stack = np.mean(frame_buffer, axis=0).astype(np.uint8)
    gray = cv2.cvtColor(avg_stack, cv2.COLOR_BGR2GRAY)

    # C. 调参流水线
    c_clip = cv2.getTrackbarPos("CLAHE", "Tuning_Panel")
    if c_clip > 0:
        clahe = cv2.createCLAHE(clipLimit=c_clip, tileGridSize=(8,8))
        gray = clahe.apply(gray)

    m_blur = cv2.getTrackbarPos("Median", "Tuning_Panel") * 2 + 1
    gray = cv2.medianBlur(gray, m_blur)

    th_val = cv2.getTrackbarPos("Threshold", "Tuning_Panel")
    _, binary = cv2.threshold(gray, th_val, 255, cv2.THRESH_BINARY_INV)

    m_size = cv2.getTrackbarPos("Morph_Size", "Tuning_Panel")
    if m_size > 0:
        kernel = np.ones((m_size, m_size), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel) # 开运算去噪

    # D. 核心：分块进行中心判定过滤 (修复了 NameError)
    final_clean_list = []
    c_dist = cv2.getTrackbarPos("Center_Dist", "Tuning_Panel")
    
    for i in range(4):
        # 切分当前盒子的二值图像
        seg = binary[i*TH : (i+1)*TH, 0:TW]
        # 调用过滤函数
        clean_seg = filter_by_center_logic(seg, dist_threshold=c_dist)
        final_clean_list.append(clean_seg)

    processed_final = cv2.vconcat(final_clean_list)

    # E. 显示结果
    cv2.imshow("1_Enhanced_Gray", gray)
    cv2.imshow("2_After_Center_Filter", processed_final)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()