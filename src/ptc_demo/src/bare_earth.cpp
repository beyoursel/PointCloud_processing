#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>


void PMF_Segment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered) {


    pcl::PointIndicesPtr ground(new pcl::PointIndices);
    // Create the filtering object
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud(cloud);
    pmf.setMaxWindowSize(20); // 最大窗口尺寸，对于较大地形可以色和之更大的最大窗口尺寸。一般最大窗口尺寸比初始窗口尺寸大一个数量级以上。
    pmf.setSlope(1.0f);
    pmf.setInitialDistance(0.5f);
    pmf.setMaxDistance(3.0f);
    pmf.extract(ground->indices);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(ground);
    extract.filter(*cloud_filtered);

    std::cerr << "Ground cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

}

int main() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);


    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path of the pcd file
    reader.read<pcl::PointXYZ>("/media/taole/Elements/daily_work/work_tg/ros_ws/data_for_seg/samp11-utm.pcd", *cloud);
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    PMF_Segment(cloud, cloud_filtered);

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("/media/taole/Elements/daily_work/work_tg/ros_ws/seg_result/samp11-utm_seged_pfm.pcd", *cloud_filtered, false);



    // Extract non-ground returns
    // extract.setNegative(true);
    // extract.filter(*cloud_filtered);

    // std::cerr << "Object cloud after filtering: " << std::endl;
    // std::cerr << *cloud_filtered << std::endl;

    // writer.write<pcl::PointXYZ> ("samp11-utm_object.pcd", *cloud_filtered, false);

    return (0);







}