import open3d as o3d
import  numpy as np
#给定三个空间点，计算平面表达式Ax+By+Cz+D=0
def computePlane(a,b,c):
        x1 = a[0]
        x2 = b[0]
        x3 = c[0]
        y1 = a[1]
        y2 = b[1]
        y3 = c[1]
        z1 = a[2]
        z2 = b[2]
        z3 = c[2]
        A = y1 * (z2 - z3) + y2 * (z3 - z1) + y3 * (z1 - z2)
        B = z1 * (x2 - x3) + z2 * (x3 - x1) + z3 * (x1 - x2)
        C = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)
        D = -(x1*(y2*z3 - y3 * z2) + x2 * (y3*z1 - y1 * z3) + x3 * (y1*z2 - y2 * z1))
        return [A,B,C,D]
if __name__ == '__main__':
   pcd=o3d.io.read_point_cloud("cow2.pcd")
   #点个数
   num_points = len(pcd.points)
   #计算平面参数
   A,B,C,D = computePlane(pcd.points[5566], pcd.points[1713], pcd.points[16046])
   #两个平面
   d1=D*1.2
   d2=D*0.8
   idx=[]
   for i,point in enumerate(pcd.points):
       #处在两个平面之间 (Ax+By+Cz+D1)  *   （Ax+By+Cz+D2）  <    0
       #请补充判定条件与操作 ⬇
       x, y, z = point
       if (A * x + B * y + C * z + d1) * (A * x + B * y + C * z + d2) < 0:idx.append(i)
        #请补充判定条件与操作 ⬆
    #提取切片的点
   slice= pcd.select_by_index(idx)
   slice.paint_uniform_color([1,0,0])
   #提取剩余的点
   rest_points= pcd.select_by_index(idx,invert=True)
   rest_points.paint_uniform_color([0,0,1])
   o3d.visualization.draw_geometries([slice])
   o3d.visualization.draw_geometries([rest_points])
   o3d.visualization.draw_geometries([slice,rest_points])
   o3d.io.write_point_cloud("4.pcd",slice)
   o3d.io.write_point_cloud("5.pcd",rest_points)