import open3d as o3d
import numpy as np

# 读取PCD文件
pcd = o3d.io.read_point_cloud("/media/taole/Elements/daily_work/work_tg/ros_ws/hill_shape.pcd")

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