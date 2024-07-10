#include <omp.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <vector>

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

    for (int i = 0; i < num_blocks; ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr block(new pcl::PointCloud<pcl::PointXYZ>);
        blocks.push_back(block);
    }

    for (auto &point : *cloud) {
        int block_idx = (point.x - min_pt.x) / x_range * num_blocks;
        blocks[block_idx]->points.push_back(point);
    }
}

int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return (-1);
    }

    // 分割点云数据
    int num_blocks = 4; // 假设分成4块
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> blocks;
    splitPointCloud(cloud, num_blocks, blocks);

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < num_blocks; ++i) {
        *filtered_cloud += *filtered_blocks[i];
    }

    // 保存过滤后的点云
    pcl::io::savePCDFile("filtered_cloud.pcd", *filtered_cloud);

    return 0;
}
