import cv2
import json
import numpy as np

# 配置参数
CONF_FILE = 'l-roi_config.json'
NUM_ROIS = 4
TARGET_POINTS = 4  # 每个盒子点4个点（左上, 右上, 右下, 左下）

# 全局变量
all_rois = []
temp_points = []
current_roi_idx = 0

def mouse_callback(event, x, y, flags, param):
    global temp_points
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(temp_points) < TARGET_POINTS:
            temp_points.append([x, y])

# 1. 初始化摄像头
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

cv2.namedWindow("ROI_Task_Panel")
cv2.setMouseCallback("ROI_Task_Panel", mouse_callback)

print(f"开始标注流程，共需设置 {NUM_ROIS} 个区域。")

while current_roi_idx < NUM_ROIS:
    ret, frame = cap.read()
    if not ret:
        break

    display_frame = frame.copy()
    
    # --- 绘制逻辑 ---
    # 1. 绘制已经固定下来的 ROI (绿色)
    for i, roi in enumerate(all_rois):
        pts = np.array(roi, np.int32)
        cv2.polylines(display_frame, [pts], True, (0, 255, 0), 2)
        cv2.putText(display_frame, f"Saved: ROI {i+1}", tuple(roi[0]), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # 2. 绘制当前正在操作的 ROI (红色)
    for pt in temp_points:
        cv2.circle(display_frame, tuple(pt), 6, (0, 0, 255), -1)
    if len(temp_points) > 1:
        pts = np.array(temp_points, np.int32)
        cv2.polylines(display_frame, [pts], False, (0, 0, 255), 2)

    # 3. 状态栏信息
    status_text = f"STEP {current_roi_idx + 1}/{NUM_ROIS}: Click 4 points for Box {current_roi_idx + 1}"
    cv2.rectangle(display_frame, (0, 0), (1280, 50), (0, 0, 0), -1)
    cv2.putText(display_frame, status_text, (20, 35), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    
    help_text = "[Space]: Confirm | [r]: Reset Current | [ESC]: Quit"
    cv2.putText(display_frame, help_text, (850, 35), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

    cv2.imshow("ROI_Task_Panel", display_frame)

    key = cv2.waitKey(1) & 0xFF

    # 按空格保存当前这一个 ROI，进入下一个
    if key == ord(' '):
        if len(temp_points) == TARGET_POINTS:
            all_rois.append(temp_points)
            temp_points = []
            current_roi_idx += 1
            print(f"成功保存第 {current_roi_idx} 个 ROI。")
        else:
            print(f"请先点满 {TARGET_POINTS} 个点！")

    # 按 'r' 重置当前选的点
    elif key == ord('r'):
        temp_points = []
        print(f"已重置当前第 {current_roi_idx + 1} 个 ROI 的坐标。")

    # 按 ESC 退出
    elif key == 27:
        break

# 2. 完成后的处理
if len(all_rois) > 0:
    with open(CONF_FILE, 'w') as f:
        json.dump({"rois": all_rois, "resolution": [1280, 720]}, f, indent=4)
    print(f"\n全部完成！配置已写入 {CONF_FILE}")
else:
    print("\n中途退出，未保存任何数据。")

cap.release()
cv2.destroyAllWindows()