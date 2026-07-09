import cv2
import numpy as np
import json
import os

# --- 全局配置 ---
CONFIG_FILE = 'tools/robot opencv/l-roi_config.json'
TW, TH = 300, 150  # 矫正后每个盒子的显示尺寸
W, H = 1280, 720   # 摄像头真实分辨率
DISPLAY_SCALE = 1.2 # 画面缩小倍数（为了在屏幕上放得下）

# 全局变量
rois = []
selected_pt = None # (roi_idx, pt_idx)

def load_config():
    global rois
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, 'r') as f:
            data = json.load(f)
            rois = data['rois']
    else:
        for i in range(3):
            offset = 250 * i + 50
            rois.append([[offset, 300], [offset, 400], [offset+200, 400], [offset+200, 300]])

def mouse_callback(event, x, y, flags, param):
    global rois, selected_pt
    
    # 【核心修复】：将鼠标在缩小窗口上的坐标还原为原始图片的坐标
    real_x = int(x * DISPLAY_SCALE)
    real_y = int(y * DISPLAY_SCALE)
    
    if event == cv2.EVENT_LBUTTONDOWN:
        for r_idx, roi in enumerate(rois):
            for p_idx, pt in enumerate(roi):
                # 增大判定范围到 20 像素，更好抓取
                if np.hypot(real_x - pt[0], real_y - pt[1]) < 20:
                    selected_pt = (r_idx, p_idx)
                    return

    elif event == cv2.EVENT_MOUSEMOVE:
        if selected_pt is not None:
            r_idx, p_idx = selected_pt
            # 限制坐标不超出边界
            rois[r_idx][p_idx] = [max(0, min(W, real_x)), max(0, min(H, real_y))]

    elif event == cv2.EVENT_LBUTTONUP:
        selected_pt = None

# --- 主程序 ---
load_config()

cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)

# 确保窗口名称一致
win_name = "Main_Tracker_Drag_the_Red_Dots"
cv2.namedWindow(win_name)
cv2.setMouseCallback(win_name, mouse_callback)

print("\n--- 拖拽操作说明 ---")
print("1. 请用鼠标左键按住红色圆点进行移动")
print("2. 移动时，右侧矫正窗口会实时跟随变化")
print("3. 调整满意后按 's' 保存，按 'q' 退出")

while True:
    ret, frame = cap.read()
    if not ret: break

    display_frame = frame.copy()
    warped_list = []

    for i, pts_list in enumerate(rois):
        src_pts = np.array(pts_list, dtype=np.float32)
        
        # 绘制：原图上的框和点
        cv2.polylines(display_frame, [src_pts.astype(np.int32)], True, (0, 255, 0), 2)
        for pt in pts_list:
            cv2.circle(display_frame, tuple(pt), 8, (0, 0, 255), -1) # 放大圆点方便看

        # 执行透视变换
        dst_pts = np.array([[0, 0], [0, TH], [TW, TH], [TW, 0]], dtype=np.float32)
        matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
        warped = cv2.warpPerspective(frame, matrix, (TW, TH))
        
        cv2.putText(warped, f"BOX {i+1}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        warped_list.append(warped)

    # 显示矫正效果
    combined_warped = cv2.vconcat(warped_list)
    cv2.imshow("Realtime_Correction", combined_warped)

    # 缩小显示主画面（这里应用了 DISPLAY_SCALE）
    small_main = cv2.resize(display_frame, (int(W/DISPLAY_SCALE), int(H/DISPLAY_SCALE)))
    cv2.imshow(win_name, small_main)

    key = cv2.waitKey(2) & 0xFF
    if key == ord('s'):
        with open(CONFIG_FILE, 'w') as f:
            json.dump({"rois": rois, "resolution": [W, H]}, f, indent=4)
        print("配置已保存到 roi_config.json")
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()