import open3d as o3d
import copy #点云拷贝库
import numpy as np

maize = o3d.io.read_point_cloud("maize.pcd")

print("点云个数：",maize)

o3d.visualization.draw_geometries([maize])  # 可视化

# x坐标≤7的点染为红色、x坐标＞7但≤7.4的点染为绿色、x坐标＞7.4的点染为蓝色

#初始化颜色矩阵
colors = np.zeros([len(cloud.points), 3])

#修改逻辑
for i, point in enumerate(cloud.points):
    x_coord = point[0]
    if x_coord <= 7:
        colors[i] = [1, 0, 0]  #red
    elif x_coord <= 7.4:
        colors[i] = [0, 1, 0]  #green
    else:
        colors[i] = [0, 0, 1]  #blue


cloud.colors = o3d.utility.Vector3dVector(colors)


o3d.visualization.draw_geometries([cloud])