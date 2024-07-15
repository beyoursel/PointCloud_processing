#include <iostream>
#include <vector>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>

// 定义点云结构
struct Point {
    float x, y, z;
};

// 栅格结构
struct Grid {
    std::vector<Point> points;
};

// 比较函数用于按高度排序
bool compareHeight(const Point& a, const Point& b) {
    return a.z > b.z;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input_pcd_file>" << std::endl;
        return -1;
    }

    // 读取PCD文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s \n", argv[1]);
        return -1;
    }

    // 体素下采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.1f, 0.1f, 0.1f); // 设置体素网格的大小
    sor.filter(*filtered_cloud);

    // 找到点云范围
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*filtered_cloud, min_pt, max_pt);

    float x_range = max_pt.x - min_pt.x;
    float y_range = max_pt.y - min_pt.y;

    // 定义栅格尺寸
    float grid_width = 10.0f;  // 栅格的宽度
    float grid_height = 10.0f; // 栅格的高度

    // 根据点云范围确定栅格数量
    int M = static_cast<int>(x_range / grid_width) + 1;
    int N = static_cast<int>(y_range / grid_height) + 1;

    // 创建栅格
    std::vector<std::vector<Grid>> grids(M, std::vector<Grid>(N));

    // 将点云分配到栅格中
    for (const auto& point : filtered_cloud->points) {
        int grid_x = static_cast<int>((point.x - min_pt.x) / grid_width);
        int grid_y = static_cast<int>((point.y - min_pt.y) / grid_height);

        if (grid_x >= 0 && grid_x < M && grid_y >= 0 && grid_y < N) {
            grids[grid_x][grid_y].points.push_back({point.x, point.y, point.z});
        }
    }

    // 创建一个新的点云用于存储保留的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr retained_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 对每个栅格中的点云按高度排序并保留后10%的点，若不足一个则保留高度最低的一个点
    for (int i = 0; i < M; ++i) {
        for (int j = 0; j < N; ++j) {
            auto& grid_points = grids[i][j].points;
            if (!grid_points.empty()) {
                // 按高度排序
                std::sort(grid_points.begin(), grid_points.end(), compareHeight);

                // 计算需要保留的点数，若不足一个则保留高度最低的一个点
                size_t retain_count = static_cast<size_t>(grid_points.size() * 0.1);
                if (retain_count < 1) {
                    retain_count = 1;
                }

                // 保留后10%的点或一个点
                for (auto it = grid_points.end() - retain_count; it != grid_points.end(); ++it) {
                    retained_cloud->points.push_back(pcl::PointXYZ(it->x, it->y, it->z));
                }
            }
        }
    }

    // 设置点云宽高
    retained_cloud->width = retained_cloud->points.size();
    retained_cloud->height = 1;
    retained_cloud->is_dense = true;

    // 保存保留的点云为PCD文件
    pcl::io::savePCDFile("retained_points.pcd", *retained_cloud);

    std::cout << "Retained points saved to retained_points.pcd" << std::endl;

    return 0;
}
