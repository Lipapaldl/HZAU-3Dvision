import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

pcd = o3d.io.read_point_cloud("scene.pcd")
# 设置为debug调试模式
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    # -------------------密度聚类--------------------------
    labels = np.array(pcd.cluster_dbscan(eps=20,               # 邻域距离
                                         min_points=1000,          # 最小点数
                                         print_progress=True))  # 是否在控制台中可视化进度条
max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
# ---------------------保存聚类结果------------------------
for i in range(max_label + 1):
    ind = np.where(labels == i)[0]
    clusters_cloud = pcd.select_by_index(ind)
    file_name = "Dbscan_cluster" + str(i+1) + ".pcd"
    o3d.io.write_point_cloud(file_name, clusters_cloud)
# --------------------可视化聚类结果----------------------
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
o3d.visualization.draw_geometries([pcd], window_name="点云密度聚类",
                                  height=480, width=600,
                                  mesh_show_back_face=0)

