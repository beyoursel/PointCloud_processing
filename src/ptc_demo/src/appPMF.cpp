#include <iostream>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointIndicesPtr ground(new pcl::PointIndices);


    std::string pcd_file_path = "/media/taole/HHD/Doc/daily_work/work_tg/ros_ws/data_for_seg/obstacle_2407051534_h3_hill_tree_v_20c.pcd";
    // 读取PCD文件到点云对象中
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud) == -1)
    {
        ROS_ERROR("Couldn't read file %s", pcd_file_path.c_str());
        return -1;
    }


	std::cerr << "Cloud before filtering: " << cloud->points.size() << std::endl;

	auto startTime = std::chrono::steady_clock::now();

	// Create the filtering object 
	// pcl::ProgressiveMorphologicalFilter<pcl::PointXYZI> pmf;
	pcl::ApproximateProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
	pmf.setInputCloud(cloud);
	pmf.setMaxWindowSize(10); // set window size
	pmf.setSlope(1.0f); // slope 
	pmf.setInitialDistance(1.0f);
	pmf.setMaxDistance(3.0f);
	pmf.extract(ground->indices);

	// Create the filtering object 
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(ground);
	extract.filter(*cloud_filtered);

	std::cerr << "Ground cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;


    pcl::io::savePCDFileASCII("cloud_seg.pcd", *cloud_filtered);
    ROS_INFO("Saved inliers pointcloud");

	// Extract non-ground returns 
	extract.setNegative(true);
	extract.filter(*cloud_filtered);

	auto endTime      = std::chrono::steady_clock::now();
    auto ellapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

	std::cout << "Ellapse-Time: " << ellapsedTime.count() << " milliseconds." << std::endl;
	std::cerr << "Object cloud after filtering: " << cloud_filtered->points.size() << std::endl;


    pcl::io::savePCDFileASCII("cloud_outlier.pcd", *cloud_filtered);
    ROS_INFO("Saved outliers pointcloud");

	return 0;
}
