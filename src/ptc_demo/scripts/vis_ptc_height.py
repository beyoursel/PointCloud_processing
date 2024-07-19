import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 加载点云数据
pcd = o3d.io.read_point_cloud("/media/taole/HHD/Doc/daily_work/work_tg/ros_ws/fitted_surface.pcd")
points = np.asarray(pcd.points)

# 获取坐标范围
x_min, x_max = points[:, 0].min(), points[:, 0].max()
y_min, y_max = points[:, 1].min(), points[:, 1].max()
z_min, z_max = points[:, 2].min(), points[:, 2].max()

# 设置颜色映射
colors = plt.cm.viridis((points[:, 2] - z_min) / (z_max - z_min))

# 创建 3D 图形
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制点云
sc = ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=colors, s=1)

# 添加颜色条
mappable = plt.cm.ScalarMappable(cmap='viridis')
mappable.set_array(points[:, 2])
cbar = plt.colorbar(mappable, ax=ax)
cbar.set_label('Height (z) [m]')

# 设置轴标签和范围
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.set_xlim([x_min, x_max])
ax.set_ylim([y_min, y_max])
ax.set_zlim([z_min, z_max])

# 设置轴刻度格式为小数点后三位
ax.xaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: f'{x:.3f}'))
ax.yaxis.set_major_formatter(plt.FuncFormatter(lambda y, _: f'{y:.3f}'))
ax.zaxis.set_major_formatter(plt.FuncFormatter(lambda z, _: f'{z:.3f}'))

# 调整刻度尺大小
ax.tick_params(axis='both', which='major', labelsize=8)  # 更小的刻度尺

# 显示坐标范围
print(f"X range: {x_min:.3f} to {x_max:.3f} m")
print(f"Y range: {y_min:.3f} to {y_max:.3f} m")
print(f"Z range: {z_min:.3f} to {z_max:.3f} m")

# 显示图形
plt.show()
