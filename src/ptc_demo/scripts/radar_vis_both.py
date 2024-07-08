import open3d as o3d
import numpy as np

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


    # 设置原始点云的颜色（例如白色）
    original_pcd.paint_uniform_color([1, 1, 1])

    # 设置分割后的点云的颜色（例如绿色）
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
    original_pcd_path = "/media/taole/Elements/daily_work/work_tg/ros_ws/data_for_seg/obstacle_2407050853_eline_v_10c.pcd"
    segmented_pcd_path = "/media/taole/Elements/daily_work/work_tg/ros_ws/seg_result/obstacle_2407050853_eline_v_10c_seged.pcd"
    load_and_visualize_pcds(original_pcd_path, segmented_pcd_path)
