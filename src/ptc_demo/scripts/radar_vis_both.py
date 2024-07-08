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



    original_pcd.paint_uniform_color([0, 1, 0])


    segmented_pcd.paint_uniform_color([1, 0, 0])

    # 创建可视化对象并添加两个点云
    # vis = o3d.visualization.Visualizer()
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="Open3D Point Cloud Viewer", width=800, height=600)

    vis.get_render_option().point_size = 1
    vis.get_render_option().background_color = np.asanyarray([0, 0, 0])


    def key_show_orgi_callback(vis):

        vis.remove_geometry(original_pcd)
        return True


    vis.add_geometry(original_pcd)
    vis.add_geometry(segmented_pcd)

    # 运行可视化

    vis.register_key_callback(ord("D"), key_show_orgi_callback)
    vis.run()
    # vis.destroy_window()

if __name__ == "__main__":

    if len(sys.argv) < 3:
        print(USAGE)

    original_pcd_path = sys.argv[1]
    segmented_pcd_path = sys.argv[2]
    load_and_visualize_pcds(original_pcd_path, segmented_pcd_path)
