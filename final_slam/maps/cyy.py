import cv2

# 加载原始地图
img = cv2.imread('2d.pgm', cv2.IMREAD_GRAYSCALE)

# 地图参数
resolution = 0.05  # 米/像素
origin_x = -1.669514
origin_y = -23.9265
height = img.shape[0]

# 你希望设置为可通行区域的 world/map 坐标
wx1, wy1 = 0, -2     # 左下角
wx2, wy2 = 16, -23    # 右上角

# 世界坐标 -> 像素坐标（注意y轴反向）
def world_to_pixel(wx, wy):
    px = int((wx - origin_x) / resolution)
    py = height - int((wy - origin_y) / resolution)  # 图像y轴向下
    return px, py

x1, y1 = world_to_pixel(wx1, wy1)
x2, y2 = world_to_pixel(wx2, wy2)

# 保证左上角在前，右下角在后
x_min, x_max = min(x1, x2), max(x1, x2)
y_min, y_max = min(y1, y2), max(y1, y2)

# 画白色填充矩形（255 = free space）
cv2.rectangle(img, (x_min, y_min), (x_max, y_max), 255, thickness=-1)

# 保存修改后地图
cv2.imwrite('2d_modified.pgm', img)

print("Done. 修改后的地图已保存为 2d_modified.pgm")
