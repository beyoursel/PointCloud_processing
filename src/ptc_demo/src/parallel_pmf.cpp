#include <omp.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <vector>
#include <iostream>
#include <chrono>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>


void VoxelDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered) {

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.3f, 0.3f, 0.3f);
    vg.filter(*voxel_filtered);

    ROS_INFO("the number of v-downsampled ptc is %ld", voxel_filtered->size());

}

// 分块处理函数
void processBlock(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_block, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_block) {
    pcl::PointIndicesPtr ground(new pcl::PointIndices);
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud(input_block);
    pmf.setMaxWindowSize(2.5);
    pmf.setSlope(1.0f);
    pmf.setInitialDistance(1.0f);
    pmf.setMaxDistance(3.0f);
    pmf.extract(ground->indices);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input_block);
    extract.setIndices(ground);
    extract.filter(*output_block);
}

void splitPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int num_blocks, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &blocks) {
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    float x_range = max_pt.x - min_pt.x;
    float y_range = max_pt.y - min_pt.y;
    float z_range = max_pt.z - min_pt.z;
    // std::cout << x_range << std::endl;
    // std::cout << y_range << std::endl;
    // std::cout << z_range << std::endl;    

    for (int i = 0; i < num_blocks; ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr block(new pcl::PointCloud<pcl::PointXYZ>);
        blocks.push_back(block);
    }

    for (auto &point : *cloud) {
        int block_idx = std::min(static_cast<int>((point.x - min_pt.x) / x_range * num_blocks), num_blocks - 1);
        blocks[block_idx]->points.push_back(point);
    }
}



void parallel_PMF(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_seg, int num_blocks){

    std::cout << cloud->size() << std::endl;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> blocks;
    splitPointCloud(cloud, num_blocks, blocks);
    std::cout << blocks[0]->size() << std::endl;    

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filtered_blocks(num_blocks);
    for (int i = 0; i < num_blocks; ++i) {
        filtered_blocks[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    // 并行处理每个块
    #pragma omp parallel for
    for (int i = 0; i < num_blocks; ++i) {
        processBlock(blocks[i], filtered_blocks[i]);
    }

    // 合并结果
    for (int i = 0; i < num_blocks; ++i) {
        *cloud_seg += *filtered_blocks[i];
    }


}


int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud_new) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return (-1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto start = std::chrono::high_resolution_clock::now();
    VoxelDownSample(cloud_new, cloud);

    // 分割点云数据
    int num_blocks = 16; // 假设分成4块
    parallel_PMF(cloud, filtered_cloud, num_blocks);

    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    std::cout << "Processing time: " << duration << " milliseconds" << std::endl;   

    // 保存过滤后的点云
    pcl::io::savePCDFile("filtered_cloud.pcd", *filtered_cloud);

    return 0;
}
