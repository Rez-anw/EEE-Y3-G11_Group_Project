import cv2
import numpy as np
import os
from scipy.spatial import cKDTree

# 1. 读取图像
image = cv2.imread('image3.jpg')  # 替换为你的图像路径

# 2. 转换为灰度图像
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# 3. 应用高斯模糊减少噪声
blurred = cv2.GaussianBlur(gray, (5, 5), 0)

# 4. 边缘检测
edges = cv2.Canny(blurred, 50, 150)

# 5. 轮廓检测
contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# 创建一个空白图像用于显示筛选的蜿蜒路径
selected_path_image = np.zeros_like(image)

# 筛选符合条件的蜿蜒路径
for cnt in contours:
    area = cv2.contourArea(cnt)
    perimeter = cv2.arcLength(cnt, True)

    if 100 < area < 2000 and perimeter > 200:
        approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)
        if len(approx) > 5:
            cv2.drawContours(selected_path_image, [cnt], -1, (255, 255, 255), 2)  # 白色路径

# 6. 检测灰色小球的位置
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# 定义灰色的HSV范围
lower_gray = np.array([35, 15, 50])
upper_gray = np.array([180, 50, 200])

# 创建灰色掩码
mask = cv2.inRange(hsv, lower_gray, upper_gray)

# 检测灰色区域轮廓
ball_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 绘制小球的位置
for cnt in ball_contours:
    x, y, w, h = cv2.boundingRect(cnt)
    if 10 < w < 50 and 10 < h < 50:
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)  # 红色框出小球

# 7. 直接处理路径骨架化
gray_selected_path = cv2.cvtColor(selected_path_image, cv2.COLOR_BGR2GRAY)

# 二值化处理
_, binary = cv2.threshold(gray_selected_path, 127, 255, cv2.THRESH_BINARY)

# 骨架化处理（需要 OpenCV 的 ximgproc 模块）
skeleton = cv2.ximgproc.thinning(binary)

# 8. 处理小球分布和连接
non_zero_points = np.column_stack(np.where(skeleton > 0))  # 获取非零点坐标

# KDTree 加速小球放置
kdtree = cKDTree(non_zero_points)
placed_points = []
used_indices = set()
ball_radius = 10  # 小球半径，确保与绘制时一致

for idx, point in enumerate(non_zero_points):
    if idx not in used_indices:
        placed_points.append(point)
        indices = kdtree.query_ball_point(point, r=25)
        used_indices.update(indices)

# 9. 绘制连接路径
connection_image = np.zeros_like(image)
placed_points = np.array(placed_points)

# 连接最近邻点
kdtree_points = cKDTree(placed_points)
visited = set()
for i, point in enumerate(placed_points):
    distances, indices = kdtree_points.query(point, k=3)
    for idx in indices[1:3]:
        if (i, idx) not in visited and (idx, i) not in visited:
            nearest_point = placed_points[idx]
            cv2.line(connection_image, (int(point[1]), int(point[0])), (int(nearest_point[1]), int(nearest_point[0])), (0, 255, 0), 2)
            visited.add((i, idx))

# 10. 创建最终的综合图像
final_output = image.copy()

# **11. 生成仅包含小球的图像**
placed_balls_image = np.zeros_like(image)

# 在原始图像上叠加连接点和路径
for i, point in enumerate(placed_points):
    cv2.circle(final_output, (int(point[1]), int(point[0])), ball_radius, (255, 0, 0), -1)  # 蓝色小球
    cv2.circle(placed_balls_image, (int(point[1]), int(point[0])), ball_radius, (255, 255, 0), -1)  # **黄色小球**

for i, point in enumerate(placed_points):
    distances, indices = kdtree_points.query(point, k=3)
    for idx in indices[1:3]:
        nearest_point = placed_points[idx]
        cv2.line(final_output, (int(point[1]), int(point[0])), (int(nearest_point[1]), int(nearest_point[0])), (0, 255, 0), 2)  # 绿色连接线

# 12. 结果合成与显示
combined_image = cv2.addWeighted(selected_path_image, 0.7, image, 0.3, 0)

# 创建输出文件夹
output_dir = "Path_output"
os.makedirs(output_dir, exist_ok=True)

# 保存图像
cv2.imwrite(os.path.join(output_dir, 'output_edges.png'), edges)
cv2.imwrite(os.path.join(output_dir, 'output_selected_path.png'), selected_path_image)
cv2.imwrite(os.path.join(output_dir, 'output_combined_result.png'), combined_image)
cv2.imwrite(os.path.join(output_dir, 'connected_points_image.png'), connection_image)
cv2.imwrite(os.path.join(output_dir, 'final_output.png'), final_output)  # 综合图像
cv2.imwrite(os.path.join(output_dir, 'placed_balls.png'), placed_balls_image)  # **确保小球大小一致的图像**

# 显示结果
cv2.imshow('Original Image', image)
cv2.imshow('Edges', edges)
cv2.imshow('Selected Path', selected_path_image)
cv2.imshow('Combined Result', combined_image)
cv2.imshow('Connected Points', connection_image)
cv2.imshow('Final Output', final_output)  # 综合图像
cv2.imshow('Placed Balls', placed_balls_image)  # **显示小球图像**

cv2.waitKey(0)
cv2.destroyAllWindows()
#Currently, the system can roughly generate paths and detect ball positions for a single image. The next steps are:  
#1. Generate path direction.  
#2. Detect ball positions in real-time video.  
#3. Use Kalman filtering to predict the ball’s next position.  
#4. Compare the path direction with the predicted ball movement direction to determine the adjustment angle.
