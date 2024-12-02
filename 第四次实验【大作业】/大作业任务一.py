import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
file_name = "大作业任务一.pcd"
pcd = o3d.io.read_point_cloud(file_name)
pcd.paint_uniform_color([1,0,0])
# o3d.visualization.draw_geometries_with_editing([pcd])

#半径滤波
cl, ind = pcd.remove_radius_outlier(nb_points=100,radius=0.8)
sor_cloud = pcd.select_by_index(ind)
# o3d.visualization.draw_geometries([sor_cloud], window_name="半径滤波")

#统计学离群点滤波
cl, ind2 = sor_cloud.remove_statistical_outlier(nb_neighbors=5,std_ratio=0.25)
sor_cloud2 = sor_cloud.select_by_index(ind2)
# o3d.visualization.draw_geometries([sor_cloud2], window_name="统计滤波")

#RANSAC平面分割
cl,ind3 = sor_cloud2.segment_plane(0.09,10,8)
sor_cloud3 = sor_cloud2.select_by_index(ind3)
Osor_cloud3 = sor_cloud2.select_by_index(ind3,invert=True)

#DBSCAN聚类 进一步分割红色和重叠部分
# 设置为debug调试模式
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    # -------------------密度聚类--------------------------
    labels = np.array(sor_cloud3.cluster_dbscan(eps=0.1,               # 邻域距离
                                         min_points=1,          # 最小点数
                                         print_progress=True))  # 是否在控制台中可视化进度条
max_label = labels.max()
# ---------------------保存聚类结果------------------------
for i in range(max_label + 1):
    ind = np.where(labels == i)[0]
    clusters_cloud = sor_cloud3.select_by_index(ind)
    file_name = "红色一进步分割" + str(i+1) + ".pcd"
    o3d.io.write_point_cloud(file_name, clusters_cloud)
# --------------------可视化聚类结果----------------------
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
sor_cloud3.colors = o3d.utility.Vector3dVector(colors[:, :3])
o3d.visualization.draw_geometries([sor_cloud3], window_name="点云DBSCAN分割")

#读取分割后然后合并
sor_cloud3_new = o3d.io.read_point_cloud('红色一进步分割1.pcd')
Osor_cloud3_new = Osor_cloud3  + o3d.io.read_point_cloud('红色一进步分割2.pcd')
Osor_cloud3_new.paint_uniform_color([1,1,0])
sor_cloud3_new.paint_uniform_color([1,0,0])

o3d.visualization.draw_geometries([sor_cloud3_new,Osor_cloud3_new])
o3d.io.write_point_cloud("plane.pcd", sor_cloud3_new)
o3d.io.write_point_cloud("line.pcd", Osor_cloud3_new)

o3d.io.write_point_cloud('任务一最终结果.pcd', sor_cloud3_new+Osor_cloud3_new)