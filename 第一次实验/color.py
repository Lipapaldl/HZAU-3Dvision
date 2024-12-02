import open3d as o3d
import numpy as np

cloud = o3d.io.read_point_cloud("maize.pcd")
cloud.paint_uniform_color([0,1,0])
o3d.visualization.draw_geometries([cloud])
pts = np.asarray(cloud.points)  # 点云转换为数组
print("点云矩阵")
print(pts)

# 改变轴
axis = 2  #### 作用为何？####
# 平分程度
avg = 3  #### 作用为何？####
# 根据高度生成色彩
colors = np.zeros([pts.shape[0], 3])  # 生成一个空的与点云相同大小的ndarray



height_max = np.max(pts[:, axis])  # z轴最大

height_min = np.min(pts[:, axis])  # z轴最小

delta_c = abs(height_max - height_min) / (255 * avg)
for j in range(pts.shape[0]):
    color_n = (pts[j, axis] - height_min) / delta_c
    if color_n <= 255:
        colors[j, :] = [0, 1 - color_n / 255, 1]
    else:
        colors[j, :] = [(color_n - 255) / 255, 0, 1]
print("颜色矩阵")
print(colors)
print("点云矩阵+颜色矩阵")
print(np.concatenate((pts,colors),axis=1))
cloud.colors = o3d.utility.Vector3dVector(colors)  # 给点云逐点赋色
o3d.visualization.draw_geometries([cloud])  # 可视化
