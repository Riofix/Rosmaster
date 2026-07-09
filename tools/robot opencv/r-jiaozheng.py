import cv2
import numpy as np
import json
import os

# --- 1. 配置 ---
CONFIG_FILE = 'r-roi_config.json'
TW, TH = 300, 150  # 矫正后的显示尺寸
W, H = 1280, 720   # 摄像头原始分辨率
WIN_NAME = "Right_Camera_Adjuster"

rois = []
selected_pt = None 

# --- 2. 加载配置 ---
def load_config():
    global rois
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, 'r') as f:
            data = json.load(f)
            rois = data['rois']
            print("成功载入右摄像头 ROI 配置")
    else:
        # 如果没有配置，在画面中间生成 4 个默认框
        for i in range(3):
            x = 200 + i * 200
            rois.append([[x, 350], [x, 380], [x+100, 380], [x+100, 350]])

# --- 3. 动态坐标映射回调 ---
def mouse_callback(event, x, y, flags, param):
    global rois, selected_pt
    
    # 【核心修复】：实时获取窗口的实际显示尺寸，计算缩放比
    # 这样无论你手动拉大还是缩小窗口，点击永远精准
    rect = cv2.getWindowImageRect(WIN_NAME)
    win_w, win_h = rect[2], rect[3]
    if win_w <= 0 or win_h <= 0: return
    
    real_x = int(x * (W / win_w))
    real_y = int(y * (H / win_h))
    
    if event == cv2.EVENT_LBUTTONDOWN:
        # 判定半径放大到 30 像素（针对你图中极小的区域）
        for r_idx, roi in enumerate(rois):
            for p_idx, pt in enumerate(roi):
                dist = np.hypot(real_x - pt[0], real_y - pt[1])
                if dist < 30: 
                    selected_pt = (r_idx, p_idx)
                    print(f"成功抓取点: ROI {r_idx+1}, Point {p_idx+1}")
                    return

    elif event == cv2.EVENT_MOUSEMOVE:
        if selected_pt is not None:
            r_idx, p_idx = selected_pt
            rois[r_idx][p_idx] = [max(0, min(W, real_x)), max(0, min(H, real_y))]

    elif event == cv2.EVENT_LBUTTONUP:
        selected_pt = None

# --- 4. 主程序 ---
load_config()
cap = cv2.VideoCapture(2) # 右摄像头
cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)

# 使用 WINDOW_NORMAL 模式，允许你手动拉大窗口进行精细操作
cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)
cv2.setMouseCallback(WIN_NAME, mouse_callback)

print("\n--- 右摄像头交互说明 ---")
print("1. [重要] 你可以手动拉大窗口，看得更清楚，拖拽也更准")
print("2. 左键按住红点拖动")
print("3. 按 's' 保存，按 'q' 退出")

while True:
    ret, frame = cap.read()
    if not ret: break

    # --- 新增：将画面旋转 180° ---
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    # -----------------------------

    display_frame = frame.copy()
    warped_list = []

    for i, pts_list in enumerate(rois):
        src_pts = np.array(pts_list, dtype=np.float32)
        
        # 绘制原图标注
        cv2.polylines(display_frame, [src_pts.astype(np.int32)], True, (0, 255, 0), 2)
        for pt_idx, pt in enumerate(pts_list):
            color = (0, 255, 255) if selected_pt == (i, pt_idx) else (0, 0, 255)
            cv2.circle(display_frame, tuple(pt), 6, color, -1)

        # 实时矫正预览
        dst_pts = np.array([[0, 0], [0, TH], [TW, TH], [TW, 0]], dtype=np.float32)
        matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
        warped = cv2.warpPerspective(frame, matrix, (TW, TH))
        cv2.putText(warped, f"BOX {i+1}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        warped_list.append(warped)

    # 显示结果
    cv2.imshow("Realtime_Correction", cv2.vconcat(warped_list))
    cv2.imshow(WIN_NAME, display_frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):
        with open(CONFIG_FILE, 'w') as f:
            json.dump({"rois": rois, "resolution": [W, H]}, f, indent=4)
        print("配置已保存到 r-roi_config.json")
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()