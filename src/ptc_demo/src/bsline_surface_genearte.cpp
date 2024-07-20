#include <iostream>
#include <vector>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <chrono>
#include <boost/filesystem.hpp>
#include <omp.h>
#include <Bspline.h>
#include <queue>
#include <limits>


void visualizePointCloudSingle(
    const vector<Eigen::Vector3d>& vertices)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr controlCloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& vertex : vertices)
    {
        cloud->points.push_back(pcl::PointXYZ(vertex[0], vertex[1], vertex[2]));
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 0, 255, 0);
    viewer->addPointCloud(cloud, cloud_color_handler, "fitted_points");

    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "control_points");

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
}


void visualizePointCloudTwo(pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cloud,
    const vector<Eigen::Vector3d>& vertices)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr controlCloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& vertex : vertices)
    {
        cloud->points.push_back(pcl::PointXYZ(vertex[0], vertex[1], vertex[2]));
    }

    // for (const auto& point : control_points)
    // {
    //     controlCloud->points.push_back(pcl::PointXYZ(point[0], point[1], point[2]));
    // }


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> downsample_color_handler(downsample_cloud, 255, 255, 255);
    viewer->addPointCloud(downsample_cloud, downsample_color_handler, "downsample_points");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 0, 255, 0);
    viewer->addPointCloud(cloud, cloud_color_handler, "fitted_points");


    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
}


void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cloud,
    const vector<Eigen::Vector3d>& vertices,
    pcl::PointCloud<pcl::PointXYZ>::Ptr  control_points)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr controlCloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& vertex : vertices)
    {
        cloud->points.push_back(pcl::PointXYZ(vertex[0], vertex[1], vertex[2]));
    }

    // for (const auto& point : control_points)
    // {
    //     controlCloud->points.push_back(pcl::PointXYZ(point[0], point[1], point[2]));
    // }


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> downsample_color_handler(downsample_cloud, 255, 255, 255);
    viewer->addPointCloud(downsample_cloud, downsample_color_handler, "downsample_points");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 0, 255, 0);
    viewer->addPointCloud(cloud, cloud_color_handler, "fitted_points");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> control_color_handler(control_points, 255, 0, 0);
    viewer->addPointCloud(control_points, control_color_handler, "control_points");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "control_points");

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
}

float getAverageHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    float temp_height = 0.0;
    for (auto& point: cloud->points) {
        temp_height += point.z;
    }
    return temp_height / cloud->points.size();
}


double time_inc(std::chrono::high_resolution_clock::time_point &t_end,
                std::chrono::high_resolution_clock::time_point &t_begin) {
  
  return std::chrono::duration_cast<std::chrono::duration<double>>(t_end -t_begin).count() * 1000;
}


int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input_pcd_file>" << std::endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s \n", argv[1]);
        return -1;
    }

    auto start = std::chrono::high_resolution_clock::now();
    // std::cout << cloud->points.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_point(new pcl::PointCloud<pcl::PointXYZ>);

    using namespace BspSurfaceRestruct;

    std::vector<Eigen::Vector3d> vertice_surface;

    bspSurface surface(cloud, 3, 10.0);

    surface.getFittingSurface(vertice_surface, 0.05);

    auto end = std::chrono::high_resolution_clock::now();
    float duration = time_inc(end, start);
    std::cout << "Processing time is: " << duration << " milliseconds" << std::endl;

    surface.Getgroundpoint(ground_point);
    // visualizePointCloudTwo(cloud, vertice_surface);

    visualizePointCloud(cloud, vertice_surface, ground_point);

    return 0;
}
