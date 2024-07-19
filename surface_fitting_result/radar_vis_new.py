import open3d as o3d
import numpy as np
import sys

USAGE = "Usage: python3 radar_vis_new.py raw_downsample.pcd  control_point.pcd fitted_surface \n"

def load_and_visualize_pcds(raw_pcd_path, control_pcd_path, fittedSuface_pcd_path):
    
    # read raw pointcloud
    raw_pcd = o3d.io.read_point_cloud(raw_pcd_path)
    if raw_pcd.is_empty():
        print(f"Failed to read file {raw_pcd_path}")
        return

    # read control point
    control_pcd = o3d.io.read_point_cloud(control_pcd_path)
    if control_pcd.is_empty():
        print(f"Failed to read file {control_pcd_path}")
        return


    # read fitted pointcloud
    fitted_pcd = o3d.io.read_point_cloud(fittedSuface_pcd_path)
    if control_pcd.is_empty():
        print(f"Failed to read file {fittedSuface_pcd_path}")
        return


    # raw pointcloud white
    raw_pcd.paint_uniform_color([1, 1, 1])

    # control point red
    control_pcd.paint_uniform_color([1, 0, 0])

    # fittedsurface green
    fitted_pcd.paint_uniform_color([0, 1, 0])

    # 将较大点云的点进行多次重复，使得其视觉上显得更大
    large_points = np.asarray(control_pcd.points)
    large_points_repeated = np.repeat(large_points, 10, axis=0)  # 这里的 5 表示重复次数
    large_pcd = o3d.geometry.PointCloud()
    large_pcd.points = o3d.utility.Vector3dVector(large_points_repeated)
    large_pcd.colors = o3d.utility.Vector3dVector(np.repeat([[1, 0, 0]], len(large_points_repeated), axis=0))

    # create vis
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="Open3D Point Cloud Viewer", width=800, height=600)

    # add render option
    vis.get_render_option().point_size = 3  # 设置点的大小
    vis.get_render_option().background_color = np.asarray([0, 0, 0])  # 设置背景颜色

    # callback
    def key_show_orgi_callback(vis):
        vis.remove_geometry(raw_pcd)
        return True

    # create TF
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10.0, origin=[0, 0, 0])

    # add TF frame to vis
    vis.add_geometry(coordinate_frame)


    # add pointcloud to vis-object
    vis.add_geometry(raw_pcd)
    vis.add_geometry(large_pcd)
    vis.add_geometry(fitted_pcd)

    # run vis
    vis.register_key_callback(ord("D"), key_show_orgi_callback)
    vis.run()

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print(USAGE)
    else:
        print(USAGE)
        raw_pcd_path = sys.argv[1]
        control_pcd_path = sys.argv[2]
        fittedSuface_pcd_path = sys.argv[3]
        load_and_visualize_pcds(raw_pcd_path, control_pcd_path, fittedSuface_pcd_path)
