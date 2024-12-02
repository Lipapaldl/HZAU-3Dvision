import open3d as o3d
import numpy as np
pcd = o3d.io.read_point_cloud("cow.pcd")
print(pcd)  # 输出点云点的个数
# o3d.visualization.draw_geometries([pcd], window_name="原始点云")
# print("Statistical oulier removal")

# 直通滤波
def pass_through_filter(pcd, axis, axis_min, axis_max):
    # 获取点云数据的Numpy数组
    points = np.asarray(pcd.points)

    # 根据指定的坐标范围筛选点云
    if axis == 'x':
        mask = (points[:, 0] > axis_min) & (points[:, 0] < axis_max)
    elif axis == 'y':
        mask = (points[:, 1] > axis_min) & (points[:, 1] < axis_max)
    elif axis == 'z':
        mask = (points[:, 2] > axis_min) & (points[:, 2] < axis_max)
    else:
        raise ValueError("Invalid axis. Axis must be 'x', 'y', or 'z'.")
 # 创建新的点云对象
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(points[mask])

    # 如果点云包含颜色信息，同样进行筛选
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
        filtered_pcd.colors = o3d.utility.Vector3dVector(colors[mask])

    # 如果点云包含法线信息，同样进行筛选
    if pcd.has_normals():
        normals = np.asarray(pcd.normals)
        filtered_pcd.normals = o3d.utility.Vector3dVector(normals[mask])

    return filtered_pcd

#统计学离群点滤波
cl, ind = pcd.remove_statistical_outlier(nb_neighbors=100,std_ratio=1)
sor_cloud = pcd.select_by_index(ind)
# o3d.visualization.draw_geometries([sor_cloud], window_name="统计滤波")

#半径滤波
cl, ind2 = sor_cloud.remove_radius_outlier(nb_points=20,radius=50)
sor_cloud2 = sor_cloud.select_by_index(ind2)
# o3d.visualization.draw_geometries([sor_cloud2], window_name="半径滤波")

#均匀下采样
# cl,ind3 = sor_cloud2.uniform_down_sample(every_k_points=50)
# sor_cloud3 = sor_cloud2.select_by_index(ind3)
# o3d.visualization.draw_geometries([sor_cloud3],window_name="均匀下采样")

n1 = 3800
n2 = 5800
pass_through = pass_through_filter(sor_cloud2,'z',n1,n2)
o3d.visualization.draw_geometries([pass_through],window_name="直通滤波")

o3d.io.write_point_cloud("Q2.pcd",pass_through)