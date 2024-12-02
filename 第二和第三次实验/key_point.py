import open3d as o3d
import numpy as np
import time


def keypoints_to_spheres(keypoints):
        spheres = o3d.geometry.TriangleMesh()
        for keypoint in keypoints.points:
         sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
         sphere.translate(keypoint)
         spheres += sphere
         spheres.paint_uniform_color([1.0, 0.0, 0.0])
        return spheres

pcd = o3d.io.read_point_cloud("body.pcd")

tic = time.time()
# --------------------ISS关键点提取的相关参数---------------------
keypoints = o3d.geometry.keypoint.compute_iss_keypoints(pcd,
                                                        salient_radius=0.01,#确定邻域搜索的半径，主要用于计算点云中每个点的局部几何结构。这个半径值越大，考虑的邻域范围越大，提取到的关键点会更加全局化。
                                                        non_max_radius=0.1,#用于非极大值抑制步骤。该半径值确定了在邻域内进行极大值抑制的范围，确保在指定范围内只有一个极大值被保留，其他极大值被抑制
                                                        gamma_21=0.8, #特征值比例阈值，用于区分主方向上的显著特征。通过设置gamma_21的值，可以过滤掉那些没有明显主方向的点。较低的gamma_21值会导致更多的点被识别为关键点。
                                                        gamma_32=0.5) #特征值比例阈值，用于区分次主方向上的显著特征。通过设置gamma_32的值，可以过滤掉那些没有明显次主方向的点。较低的gamma_32值会导致更多的点被识别为关键点。
toc = 1000 * (time.time() - tic)
print("ISS Computation took {:.0f} [ms]".format(toc))
print("Extract",keypoints)
pcd.paint_uniform_color([0.08, 0.3, 0.4])
o3d.visualization.draw_geometries([keypoints_to_spheres(keypoints), pcd],width=800,height=800)

