import open3d as o3d
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor

##
# 函数：返回提取到的平面点云
# 原理：先过滤，再分割平面
# #
def extract_planes(pcd, distance_threshold=0.09, ransac_n=3, num_iterations=100):
    # 滤除噪点
    _, ind =  pcd.remove_statistical_outlier(nb_neighbors=5, std_ratio=0.25)
    cloud = pcd.select_by_index(ind)
    o3d.visualization.draw_geometries([cloud], window_name="过滤后的点云")

    # RANSAC平面分割
    _, inliers = cloud.segment_plane(distance_threshold=distance_threshold,
                                                         ransac_n=ransac_n,
                                                         num_iterations=num_iterations)
    cloud2 = cloud.select_by_index(inliers)
    #当然要染色了
    cloud2.paint_uniform_color([1.0, 0, 0])

    o3d.visualization.draw_geometries([cloud2], window_name="分割得到的平面点云")


    return cloud2,inliers

###
# 函数：过滤得到直线
# #
def extract_lines(pcd,plane_inliers):
    # 滤除噪点
    _, ind = pcd.remove_statistical_outlier(nb_neighbors=5, std_ratio=0.25)
    cloud = pcd.select_by_index(ind)

    lines = cloud.select_by_index(plane_inliers, invert=True)

    # 当然要染色了
    lines.paint_uniform_color([1.0, 1.0, 0])

    o3d.visualization.draw_geometries([lines], window_name="分割得到的直线点云")
    return lines

# 读取点云数据
pcd = o3d.io.read_point_cloud("大作业任务一.pcd")

# 可视化原始点云
# o3d.visualization.draw_geometries([pcd], window_name="原始点云")

# 提取平面
plane,plane_inliers = extract_planes(pcd)

#提取直线
line = extract_lines(pcd,plane_inliers)

# 可视化处理后的点云
o3d.visualization.draw_geometries([plane,line], window_name="处理后的点云")

# 保存处理后的点云
o3d.io.write_point_cloud("plane.pcd", plane)
o3d.io.write_point_cloud("line.pcd", line)

o3d.io.write_point_cloud("merge_result.pcd", line+plane)