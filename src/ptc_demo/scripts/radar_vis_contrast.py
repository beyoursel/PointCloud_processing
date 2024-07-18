import open3d as o3d
import numpy as np
import sys

USAGE = "Usage: python3 radar_vis.py 1.PCD 2.PCD \n"

def load_and_visualize_pcds(original_pcd_path, segmented_pcd_path):
    # 读取原始点云
    original_pcd = o3d.io.read_point_cloud(original_pcd_path)
    if original_pcd.is_empty():
        print(f"Failed to read file {original_pcd_path}")
        return

    # 读取分割后的点云
    segmented_pcd = o3d.io.read_point_cloud(segmented_pcd_path)
    if segmented_pcd.is_empty():
        print(f"Failed to read file {segmented_pcd_path}")
        return

    # 为原始点云着色为白色
    original_pcd.paint_uniform_color([1, 1, 1])

    # 为分割后的点云着色为红色
    segmented_pcd.paint_uniform_color([1, 0, 0])

    # 将较大点云的点进行多次重复，使得其视觉上显得更大
    # large_points = np.asarray(segmented_pcd.points)
    # large_points_repeated = np.repeat(large_points, 5, axis=0)  # 这里的 5 表示重复次数
    # large_pcd = o3d.geometry.PointCloud()
    # large_pcd.points = o3d.utility.Vector3dVector(large_points_repeated)
    # large_pcd.colors = o3d.utility.Vector3dVector(np.repeat([[1, 0, 0]], len(large_points_repeated), axis=0))

    # 创建可视化对象并添加两个点云
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="Open3D Point Cloud Viewer", width=800, height=600)

    # 设置渲染选项
    vis.get_render_option().point_size = 2  # 设置点的大小
    vis.get_render_option().background_color = np.asarray([0, 0, 0])  # 设置背景颜色

    # 注册键盘回调函数
    def key_show_orgi_callback(vis):
        vis.remove_geometry(original_pcd)
        return True

    # 创建一个坐标系，尺寸为1.0
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10.0, origin=[0, 0, 0])

    # 将坐标系添加到可视化窗口中
    vis.add_geometry(coordinate_frame)


    # 添加点云到可视化对象
    vis.add_geometry(original_pcd)
    vis.add_geometry(segmented_pcd)

    # 运行可视化
    vis.register_key_callback(ord("D"), key_show_orgi_callback)
    vis.run()

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(USAGE)
    else:
        original_pcd_path = sys.argv[1]
        segmented_pcd_path = sys.argv[2]
        load_and_visualize_pcds(original_pcd_path, segmented_pcd_path)
