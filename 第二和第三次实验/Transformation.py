import open3d
import numpy as np
from copy import deepcopy
def open3d_affine():
    # 读取点云文件
    pcd_path = "bunny.pcd"
    pcd = open3d.io.read_point_cloud(pcd_path)
    pcd = open3d.geometry.PointCloud(pcd)
    pcd.paint_uniform_color(color=[0.5, 0.5, 0.5])  # 灰色
    # 仿射变换
    pcd_affine = deepcopy(pcd)
    Rotate = pcd_affine.get_rotation_matrix_from_xyz(rotation=[0, 0, np.pi])   # z轴顺时针转180°
    Translate = np.array([-0.1, 0, 0]).reshape(3, 1)  #  x轴平移0.1(向右)
    Scale = -2   # 扩大两倍
    A1 = np.concatenate([Rotate, Translate], axis=1)
    A1 = np.concatenate([A1, np.zeros([1, 4])], axis=0)
    A1[-1, -1] = Scale
    print(A1)
# [[-1.0000000e+00 -1.2246468e-16  0.0000000e+00 -1.0000000e-01]
#  [ 1.2246468e-16 -1.0000000e+00  0.0000000e+00  0.0000000e+00]
#  [ 0.0000000e+00  0.0000000e+00  1.0000000e+00  0.0000000e+00]
#  [ 0.0000000e+00  0.0000000e+00  0.0000000e+00 -2.0000000e+00]]
    pcd_affine.transform(A1)
    pcd_affine.paint_uniform_color(color=[0, 0, 1])  # 蓝色
    # 可视化点云列表
    open3d.visualization.draw_geometries([pcd, pcd_affine],
                                         window_name="affine",
                                         width=800,
                                         height=600)
open3d_affine()
