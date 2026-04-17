std_msgs/Header header
string label          # 识别的目标类名，如 "1", "2", "3"
float32 confidence    # 分类置信度 0.0 - 1.0 (如果基于模板匹配，可以是匹配度)
int32 pixel_x         # 对应的图像中心像素 x 坐标
int32 pixel_y         # 对应的图像中心像素 y 坐标
float32 offset_x      # [预留] 解算后的物理纵向水平偏移(米)  
float32 offset_y      # [预留] 解算后的物理横向水平偏移(米)
