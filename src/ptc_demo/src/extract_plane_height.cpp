#include <iostream>
#include <vector>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <chrono>
#include <boost/filesystem.hpp>
#include <omp.h>


std::string getFileNameWithoutExtension(const std::string& filePath) {
    size_t lastSlash = filePath.find_last_of("/\\");
    std::string fileNameWithExtension = filePath.substr(lastSlash + 1);
    size_t lastDot = fileNameWithExtension.find_last_of(".");
    return fileNameWithExtension.substr(0, lastDot);
}


struct Point {
    float x, y, z;
};

struct Grid {
    std::vector<Point> points;
};


bool compareHeight(const Point& a, const Point& b) {
    return a.z < b.z;
}


void StatisticalRemoveOutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud , pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered) {
      // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (input_cloud);
    sor.setMeanK (10);
    sor.setStddevMulThresh (2.0);
    sor.filter (*cloud_filtered);
}


void PassthroghFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud , pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered) {
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (input_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-5, 30);
    //pass.setNegative (true);
    pass.filter (*cloud_filtered);
    // ROS_INFO("the number of passthrough ptc is %ld", cloud_filtered->size()); 
    std::cout << "the number of passthrough ptc is "  << cloud_filtered->size() << std::endl;
}


double time_inc(std::chrono::high_resolution_clock::time_point &t_end,
                std::chrono::high_resolution_clock::time_point &t_begin) {
  
  return std::chrono::duration_cast<std::chrono::duration<double>>(t_end -t_begin).count() * 1000;
}


void getMinMax3D_Custom(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointXYZ& min_pt, pcl::PointXYZ& max_pt) {
    // 初始化 min_pt 和 max_pt
    min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<float>::max();
    max_pt.x = max_pt.y = max_pt.z = -std::numeric_limits<float>::max();

    // 遍历点云，找到最小和最大值
    for (const auto& point : cloud->points) {
        if (point.x < min_pt.x) min_pt.x = point.x;
        if (point.y < min_pt.y) min_pt.y = point.y;
        if (point.z < min_pt.z) min_pt.z = point.z;

        if (point.x > max_pt.x) max_pt.x = point.x;
        if (point.y > max_pt.y) max_pt.y = point.y;
        if (point.z > max_pt.z) max_pt.z = point.z;
    }
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

    std::string fileName = getFileNameWithoutExtension(argv[1]);

    std::string output_folder = "/media/taole/HHD/Doc/daily_work/work_tg/ros_ws/seg_result/simple_extract";
    if (!boost::filesystem::exists(output_folder)) {
        if (!boost::filesystem::create_directories(output_folder)) {
            std::cout << "Failed to create output folder" << std::endl;
            return -1;
        }
    }

    auto start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // PassthroghFilter(cloud, filtered_cloud);

    pcl::VoxelGrid<pcl::PointXYZ> sor;

    sor.setInputCloud(cloud);
    sor.setLeafSize(0.5f, 0.5f, 0.5f);  // voxel_size 
    sor.filter(*filtered_cloud);


    // StatisticalRemoveOutlier(filtered_cloud, filtered_cloud);

    // std::cout << "the number of voxel ptc is "  <<  filtered_cloud->size() << std::endl;

    // auto end3 = std::chrono::high_resolution_clock::now();
    // double duration3 = time_inc(end3, start);
    // std::cout << "passthrough and voxel: " << duration3 << " milliseconds" << std::endl;


    // auto start2 = std::chrono::high_resolution_clock::now();

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*filtered_cloud, min_pt, max_pt); //
    // getMinMax3D_Custom(filtered_cloud, min_pt, max_pt); // 耗时严重


    // auto start2 = std::chrono::high_resolution_clock::now();

    // auto end2 = std::chrono::high_resolution_clock::now();
    // double duration2 = time_inc(end2, start2);
    // std::cout << "test grid: " << duration2 << " milliseconds" << std::endl;


    // auto start1 = std::chrono::high_resolution_clock::now();

    float x_range = max_pt.x - min_pt.x;
    float y_range = max_pt.y - min_pt.y;
    // grid_size
    float grid_width = 10.0f;
    float grid_height = 10.0f;

    int M = static_cast<int>(x_range / grid_width) + 2;
    int N = static_cast<int>(y_range / grid_height) + 2;

    std::vector<std::vector<Grid>> grids(M, std::vector<Grid>(N));

    for (size_t i = 0; i < filtered_cloud->points.size(); ++i) {
        const auto& point = filtered_cloud->points[i];
        int grid_x = static_cast<int>((point.x - min_pt.x) / grid_width);
        int grid_y = static_cast<int>((point.y - min_pt.y) / grid_height);

        if (grid_x >= 0 && grid_x < M && grid_y >= 0 && grid_y < N) {

            grids[grid_x][grid_y].points.push_back({point.x, point.y, point.z});
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);


    for (int i = 0; i < M; ++i) {
        for (int j = 0; j < N; ++j) {
            auto& grid_points = grids[i][j].points;
            if (!grid_points.empty()) {

                // auto start1 = std::chrono::high_resolution_clock::now();

                std::sort(grid_points.begin(), grid_points.end(), compareHeight);
                // auto end1 = std::chrono::high_resolution_clock::now();

                // auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1).count();
                // std::cout << "processing time per grid: " << duration1 << " milliseconds" << std::endl;

                // auto start2 = std::chrono::high_resolution_clock::now();
                size_t retain_count = static_cast<size_t>(grid_points.size() * 0.1);
                if (retain_count < 1) {
                    retain_count = 1;
                }
                // one point at the bottom of 10%
                ground_cloud->points.push_back(pcl::PointXYZ((i + 0.5) * grid_width + min_pt.x, (j + 0.5) * grid_height + min_pt.y, grid_points[retain_count-1].z));

                // mean height of 10% pointcloud at bottom
                // float height_sum = 0.0;
                // for (auto it = grid_points.begin(); it != grid_points.begin() + retain_count; ++it) {
                //     height_sum += it->z;
                // }
                // ground_cloud->points.push_back(pcl::PointXYZ((i + 0.5) * grid_width + min_pt.x, (j + 0.5) * grid_height + min_pt.y, height_sum/retain_count));
                
                // for (auto it = grid_points.begin(); it != grid_points.end() - retain_count; ++it) {
                //     non_ground_cloud->points.push_back(pcl::PointXYZ(it->x, it->y, it->z));
                // }


            }
        }
    }

    // auto end1 = std::chrono::high_resolution_clock::now();  
    // double duration1 = time_inc(end1, start1);
    // std::cout << "SORT processing time: " << duration1 << " milliseconds" << std::endl;


    auto end = std::chrono::high_resolution_clock::now();   
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Extract plane processing time: " << duration << " milliseconds" << std::endl;

    StatisticalRemoveOutlier(ground_cloud, ground_cloud);

    ground_cloud->width = ground_cloud->points.size();
    ground_cloud->height = 1;
    ground_cloud->is_dense = true;

    // non_ground_cloud->width = non_ground_cloud->points.size();
    // non_ground_cloud->height = 1;
    // non_ground_cloud->is_dense = true;

    std::string ground_pcd_file_save = output_folder + "/" + fileName + "_ground.pcd";
    // std::string non_ground_pcd_file_save = output_folder + "/" + fileName + "_non_ground.pcd";
    std::string downsample_pcd_file_save = output_folder + "/" + fileName + "_downsample.pcd";

    pcl::io::savePCDFile(ground_pcd_file_save, *ground_cloud);
    pcl::io::savePCDFile(downsample_pcd_file_save, *filtered_cloud);

    // pcl::io::savePCDFile(non_ground_pcd_file_save, *non_ground_cloud);

    std::cout << "Ground points saved to " << ground_pcd_file_save << std::endl;
    // std::cout << "Non-ground points saved to " << non_ground_pcd_file_save << std::endl;

    return 0;
}
