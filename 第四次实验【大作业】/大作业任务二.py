import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import copy
import time
from copy import deepcopy
from sklearn.cluster import KMeans
from sklearn.cluster import DBSCAN
# ----------------读取点云数据--------------
pcd_A = o3d.io.read_point_cloud("大作业任务二A.pcd")
pcd_A.paint_uniform_color([1,0,0])
pcd_B = o3d.io.read_point_cloud("大作业任务二B.pcd")
# o3d.visualization.draw_geometries([pcd_A, pcd_B])  # 可视化原始结果
print(pcd_A)
print(pcd_B)

#-----------------对B进行过滤-------------
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
pass_through = pass_through_filter(pcd_B,'y',-13,150)
o3d.visualization.draw_geometries([pass_through],window_name="直通滤波")
cl, ind = pass_through.remove_statistical_outlier(nb_neighbors=500,std_ratio=0.5)
sor_cloud = pass_through.select_by_index(ind)


o3d.visualization.draw_geometries([sor_cloud],window_name="统计学离群点滤波")
o3d.visualization.draw_geometries([pcd_A, sor_cloud])
均匀下采样:保证密度类似
downPcd_A = pcd_A.uniform_down_sample(every_k_points=5)
downPcd_B = sor_cloud.uniform_down_sample(every_k_points=5)

voxel_size = 6  # 根据需要调整体素大小
downPcd_A = pcd_A.voxel_down_sample(voxel_size)
downPcd_B =  sor_cloud.voxel_down_sample(voxel_size)
o3d.visualization.draw_geometries([downPcd_A,downPcd_B],window_name="体素均匀下采样")

point = np.asarray(downPcd_A.points)  # 获取点坐标
kdtree = o3d.geometry.KDTreeFlann(downPcd_A)  # 建立KD树索引
point_size = point.shape[0]  # 获取点的个数
dd = np.zeros(point_size)
for i in range(point_size):
    [_, idx, dis] = kdtree.search_knn_vector_3d(point[i], 2)
    dd[i] = dis[1]  # 获取到最近邻点的距离平方
density = np.mean(np.sqrt(dd))  # 计算平均密度
print("点云A密度为 denstity=", density)

point = np.asarray(downPcd_B.points)  # 获取点坐标
kdtree = o3d.geometry.KDTreeFlann(downPcd_B)  # 建立KD树索引
point_size = point.shape[0]  # 获取点的个数
dd = np.zeros(point_size)
for i in range(point_size):
    [_, idx, dis] = kdtree.search_knn_vector_3d(point[i], 2)
    dd[i] = dis[1]  # 获取到最近邻点的距离平方
density = np.mean(np.sqrt(dd))  # 计算平均密度
print("点云B密度为 denstity=", density)
# #-----------------点云分割-----------------
# 设置为debug调试模式
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    # -------------------密度聚类--------------------------
    sor_cloud = copy.deepcopy(downPcd_B)
    pcd_A = copy.deepcopy(downPcd_A)
    labels = np.array(sor_cloud.cluster_dbscan(eps=25,               # 邻域距离
                                         min_points=20,          # 最小点数
                                         print_progress=True))  # 是否在控制台中可视化进度条
max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
# ---------------------保存聚类结果------------------------
for i in range(max_label + 1):
    ind = np.where(labels == i)[0]
    clusters_cloud = sor_cloud.select_by_index(ind)
    file_name = "Dbscan_cluster" + str(i+1) + ".pcd"
    o3d.io.write_point_cloud(file_name, clusters_cloud)
# --------------------可视化聚类结果----------------------
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
sor_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
o3d.visualization.draw_geometries([sor_cloud], window_name="点云密度聚类",
                                  height=480, width=600,
                                  mesh_show_back_face=0)

# # # -----------------调整大小-----------（此步骤多余）
#读取分割后的B
# pcd_B = o3d.io.read_point_cloud("Dbscan_cluster2.pcd")
# pcd_A = copy.deepcopy(downPcd_A)
# # # 缩放点云
# scale_factor = 1 # 缩放因子
# pcd_B.scale(scale_factor, center=pcd_B.get_center())
# o3d.io.write_point_cloud("调整大小后的B.pcd", pcd_B)
# o3d.visualization.draw_geometries([pcd_A,pcd_B])

#------------------点云配准
down_A = pcd_A.voxel_down_sample(voxel_size=2)
down_B = pcd_B.voxel_down_sample(voxel_size=2)
print(pcd_A)
print(pcd_B)
# print(down_A)
print(down_B)
pcd_B = copy.deepcopy(down_B)
pcd_A = copy.deepcopy(down_A)
# -------------传入点云数据，计算FPFH------------
def FPFH_Compute(pcd):
    radius_normal = 10  # kdtree参数，用于估计法线的半径，
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=50))
# 估计法线的1个参数，使用混合型的kdtree，半径内取最多30个邻居
    radius_feature = 10  # kdtree参数，用于估计FPFH特征的半径
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(pcd,
    o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=50))  # 计算FPFH特征,搜索方法kdtree
    return pcd_fpfh  # 返回FPFH特征

# ----------------RANSAC配准--------------------
def execute_global_registration(source, target, source_fpfh,
                                target_fpfh):  # 传入两个点云和点云的特征
    distance_threshold = 1  # 设定距离阈值
    print("we use a liberal distance threshold %.3f." % distance_threshold)
# 2个点云，两个点云的特征，距离阈值，一个函数，4，
# 一个list[0.9的两个对应点的线段长度阈值，两个点的距离阈值]，
# 一个函数设定最大迭代次数和最大验证次数
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source, target, source_fpfh, target_fpfh, True, distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        4, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))
    return result


# ---------------可视化配准结果----------------
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)         # 由于函数transformand paint_uniform_color会更改点云，
    target_temp = copy.deepcopy(target)         # 因此调用copy.deepcoy进行复制并保护原始点云。
    source_temp.paint_uniform_color([1, 0, 0])  # 点云着色
    target_temp.paint_uniform_color([0, 1, 0])
    source_temp.transform(transformation)
    # o3d.io.write_point_cloud("trans_of_source.pcd", source_temp)#保存点云
    o3d.visualization.draw_geometries([source_temp, target_temp],width=600, height=600)


# ----------------读取点云数据--------------
o3d.visualization.draw_geometries([pcd_A, pcd_B],width=600, height=600)  # 可视化原始结果
print(pcd_A)
print(pcd_B)
# -----------------计算的FPFH---------------
source_fpfh=FPFH_Compute(pcd_A)
target_fpfh=FPFH_Compute(pcd_B)
# ---------------调用RANSAC执行配准------------
start = time.time()
result_ransac = execute_global_registration(pcd_A, pcd_B,
                                            source_fpfh, target_fpfh)
print("Global registration took %.3f sec.\n" % (time.time() - start))
print(result_ransac)  # 输出RANSAC配准信息
Tr = result_ransac.transformation
draw_registration_result(pcd_A, pcd_B, Tr)  # 可视化配准结果
# ------------------ICP配准-------------------
start = time.time()
icp_p2plane =  o3d.pipelines.registration.registration_icp(
        pcd_A, pcd_B, 0.5, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),    # 执行点对面的ICP算法
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30))  # 设置最大迭代次数
print("ICP registration took %.3f sec.\n" % (time.time() - start))
print(icp_p2plane)  # 输出ICP相关信息
print("Transformation is:")
print(icp_p2plane.transformation)  # 输出变换矩阵
draw_registration_result(pcd_A, pcd_B, icp_p2plane.transformation)
# ------------------身高计算-------------------
def dist(point,a,b,c,d):
    """
    计算点到平面的距离
    :param point: 平面外面一点
    :param a: 平面方程系数a
    :param b: 平面方程系数b
    :param c: 平面方程系数c
    :param d: 平面方程系数d
    :return: 点到平面的距离
    """
    dis = abs(a*point[0] + b*point[1] + c*point[2] - d)/np.sqrt(a **2 + b**2 + c**2)
    return dis


def plane_equation_from_points(p1, p2, p3):
    # 计算向量
    v1 = np.array(p2) - np.array(p1)
    v2 = np.array(p3) - np.array(p1)

    # 计算叉积
    normal_vector = np.cross(v1, v2)
    a, b, c = normal_vector

    # 计算d
    d = -np.dot(normal_vector, p1)

    return a, b, c, d
a, b, c, d = plane_equation_from_points([-220,29,-610],[-250,120,-660],[-220,26,-770])
print("平面系数：%d %d %d %d",a,b,c,d)
print("恐龙身高：",dist([-19,130,-620],a,b,c,d))