import open3d as o3d


def pick_points(cloud):

    print("   Press [shift + right click] to undo point picking")
    print(" After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name='查看点坐标', width=800, height=800, left=50, top=50, visible=True)
    vis.add_geometry(cloud)
    vis.run()  # user picks points
    vis.destroy_window()
    return vis.get_picked_points()


if __name__ == "__main__":
    pcd = o3d.io.read_point_cloud("cow2.pcd")
    print("第0个点的坐标为：\n", pcd.points[0])
    pick_points(pcd)




