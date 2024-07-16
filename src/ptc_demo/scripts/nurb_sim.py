import open3d as o3d
import numpy as np
from geomdl import NURBS
from geomdl.visualization import VisMPL

# 创建一个 NURBS 曲线
curve = NURBS.Curve()
curve.degree = 3
curve.ctrlpts = [[0, 0, 0], [1, 2, 0], [2, -1, 0], [3, 3, 0]]
curve.knotvector = [0, 0, 0, 0, 1, 1, 1, 1]

# 评估曲线并获取点
curve.evaluate()
curve_points = curve.evalpts

# 将点转换为 numpy 数组
curve_points = np.array(curve_points)

# 创建 Open3D 点云
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(curve_points)

# 可视化曲线
o3d.visualization.draw_geometries([pcd])

# import os

# file_path = '/home/taole/anaconda3/envs/uav/lib/python3.8/site-packages/geomdl/visualization/VisMPL.py'

# def replace_in_file(file_path, old_string, new_string):
#     with open(file_path, 'r') as file:
#         file_data = file.read()

#     file_data = file_data.replace(old_string, new_string)

#     with open(file_path, 'w') as file:
#         file.write(file_data)

# replace_in_file(file_path, 'np.float', 'float')  # 或者 'np.float64'
