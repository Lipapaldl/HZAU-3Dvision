import open3d as o3d
import numpy as np

print("Load a pcd point cloud, print it, and render it")
pcd = o3d.io.read_point_cloud("body.pcd")
print(pcd)  # 输出点云点的个数
print(np.asarray(pcd.points))  # 输出点的三维坐标
print("Downsample the point cloud with a voxel of 0.005")
downpcd = pcd.voxel_down_sample(voxel_size=0.005)  # 下采样滤波，体素边长为0.005m
print("Recompute the normal of the downsampled point cloud")
# 计算法线，搜索半径1cm，只考虑邻域内的30个点
downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
print("Print a normal vector of the 0th point")
print(downpcd.normals[0])  # 输出0点的法向量值
print("Print the normal vectors of the first 10 points")
print(np.asarray(downpcd.normals)[:10, :])  # 输出前10个点的法向量
o3d.visualization.draw_geometries([downpcd], point_show_normal=False, window_name="法线估计")
o3d.visualization.draw_geometries([downpcd], point_show_normal=True, window_name="法线估计")  # 可视化点云和法线

