import open3d as o3d
import numpy as np
import sys


USAGE = "Usage: python3 radar_vis.py XXX.PCD\n"



if __name__ == "__main__":

    pcd_file = "/media/taole/HHD/Doc/daily_work/work_tg/ros_ws/result.pcd"
    pcd = o3d.io.read_point_cloud(pcd_file)

    # 打印点云的信息
    print(pcd)

    # 可视化点云
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()
    vis.get_render_option().point_size = 1
    vis.get_render_option().background_color = np.asanyarray([0, 0, 0])

    vis.add_geometry(pcd)

    # pcd_save = o3d.geometry.PointCloud()
    # pcd_save.points = pcd.points
    # o3d.io.write_point_cloud("hill_shape.pcd", pcd)

    vis.run()