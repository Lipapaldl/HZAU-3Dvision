import open3d as o3d
import numpy as np

# ------------------------------------读取点云---------------------------------------
pcd = o3d.io.read_point_cloud("seg_house.pcd")
# ------------------------------------参数设置---------------------------------------
segment = []    # 存储分割结果的容器
min_num = 8200  # 每个分割平面所需的最小点数
distance_threshold = 1.2   # Ransac分割的距离阈值
iters = 0       # 用于统计迭代次数，非待设置参数
# -----------------------------------分割多个平面-------------------------------------
while len(pcd.points) > min_num:
    plane_model, inliers = pcd.segment_plane(distance_threshold,
                                             ransac_n=10,
                                             num_iterations=100)
    plane_cloud = pcd.select_by_index(inliers)       # 分割出的平面点云
    r_color = np.random.uniform(0, 1, (1, 3))        # 平面点云随机赋色
    plane_cloud.paint_uniform_color([r_color[:, 0], r_color[:, 1], r_color[:, 2]])
    pcd = pcd.select_by_index(inliers, invert=True)  # 剩余的点云
    segment.append(plane_cloud)
    file_name = "RansacFitMutiPlane" + str(iters + 1) + ".pcd"
    o3d.io.write_point_cloud(file_name, plane_cloud)
    iters += 1
    if len(inliers) < min_num:
        break

# ------------------------------------结果可视化--------------------------------------
o3d.visualization.draw_geometries(segment, window_name="Ransac分割多个平面")

