#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <thread>
#include <chrono>
#include <ros/ros.h>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_segmentation_example");
    ros::NodeHandle nh;

    std::string pcd_file_path;
    if (!nh.getParam("/plane_model/pcd_file_path", pcd_file_path))
    {
        ROS_ERROR("Failed to get param 'pcd_file_path'");
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud) == -1)
    {
        ROS_ERROR("Couldn't read file %s", pcd_file_path.c_str());
        return -1;
    }

    ROS_INFO("The number of points before segmentation is %ld", cloud->size());

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
        return -1;
    }
    ROS_INFO("The number of segmented points is %ld", inliers->indices.size());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_plane);

    std::thread viewer_thread1(visualizePointCloud, cloud, "Original Point Cloud Viewer");
    std::thread viewer_thread2(visualizePointCloud, cloud_plane, "Segmented Point Cloud Viewer");

    viewer_thread1.join();
    viewer_thread2.join();

    return 0;
}
