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

// 定义点的结构体
struct Point {
    double x, y, z;

    // 比较操作符，按照 x 坐标排序
    bool operator<(const Point& other) const {
        return x < other.x;
    }
};

// 将pcl::PointCloud<pcl::PointXYZ>::Ptr转换为std::vector<Point>
std::vector<Point> pclPointCloudToVector(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::vector<Point> points;

    for (const auto& pt : cloud->points) {
        Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        points.push_back(p);
    }

    return points;
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


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 255, 255, 255);
    viewer->addPointCloud(downsample_cloud, cloud_color_handler, "downsample_points");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> downsample_color_handler(cloud, 0, 255, 0);
    viewer->addPointCloud(cloud, downsample_color_handler, "fitted_points");

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

// 插值函数：使用KNN（k近邻）算法来填充空栅格
float interpolatePoint(const std::vector<Point>& points, double x, double y, int k = 3) {
    std::priority_queue<std::pair<double, Point>> pq;

    for (const auto& pt : points) {
        double dist = std::sqrt((pt.x - x) * (pt.x - x) + (pt.y - y) * (pt.y - y));
        pq.push(std::make_pair(dist, pt));
        if (pq.size() > k) {
            pq.pop();
        }
    }

    double sum_z = 0.0;
    int count = 0;
    while (!pq.empty()) {
        auto top = pq.top();
        pq.pop();
        // sum_x += top.second.x;
        // sum_y += top.second.y;
        sum_z += top.second.z;
        count++;
    }
    return sum_z / count;
    // return Eigen::Vector3d(sum_x / count, sum_y / count, sum_z / count);
}


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

    // auto start = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // PassthroghFilter(cloud, filtered_cloud);

    pcl::VoxelGrid<pcl::PointXYZ> sor;

    sor.setInputCloud(cloud);
    sor.setLeafSize(0.5f, 0.5f, 0.5f);  // voxel_size 
    sor.filter(*filtered_cloud);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*filtered_cloud, min_pt, max_pt); //


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

    for (int i = 0; i < M; ++i) {
        for (int j = 0; j < N; ++j) {
            auto& grid_points = grids[i][j].points;
            if (!grid_points.empty()) {


                std::sort(grid_points.begin(), grid_points.end(), compareHeight);

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
            }
        }
    }

    StatisticalRemoveOutlier(ground_cloud, ground_cloud);

    auto start = std::chrono::high_resolution_clock::now();

    pcl::PointXYZ min_pt_g, max_pt_g;
    pcl::getMinMax3D(*ground_cloud, min_pt_g, max_pt_g);

    float x_range_g = max_pt_g.x - min_pt_g.x;
    float y_range_g = max_pt_g.y - min_pt_g.y;


    int M_g = static_cast<int>(x_range_g / grid_width) + 1;
    int N_g = static_cast<int>(y_range_g / grid_height) + 1;    

    vector<vector<Eigen::Vector3d>> cnPoint(M_g, vector<Eigen::Vector3d>(N_g));
    double maxDouble = std::numeric_limits<double>::max();

    // create control points grid
    for (size_t i = 0; i < M_g; i++) {
        for (size_t j = 0; j < N_g; j++) {
            // cnPoint[i][j] = Eigen::Vector3d(min_pt.x + i * grid_width, min_pt.y + j * grid_height, low_peak);
            cnPoint[i][j](0) = min_pt_g.x + i * grid_width;
            cnPoint[i][j](1) = min_pt_g.y + j * grid_height;
            cnPoint[i][j](2) = maxDouble;
        }
    }

    for (const auto& point : ground_cloud->points) {
        float x_ptc = static_cast<float>(point.x);
        float y_ptc = static_cast<float>(point.y);
        cnPoint[(x_ptc - min_pt_g.x) / grid_width][(y_ptc - min_pt_g.y) / grid_height](2) = point.z;
    }

    std::vector<Point> ptc = pclPointCloudToVector(ground_cloud);
    // // 插值填充空栅格
    for (int i = 0; i < M_g; ++i) {
        for (int j = 0; j < N_g; ++j) {
            if (cnPoint[i][j](2) == maxDouble) {
                double x_em = min_pt_g.x + i * grid_width;
                double y_em = min_pt_g.y + j * grid_height;
                cnPoint[i][j](2) = interpolatePoint(ptc, x_em, y_em);
            }
        }
    }

    int ku = 3; // u 向的阶数
    int kv = 3; // v 向的阶数

    // 设置均匀节点向量
    vector<float> knots_u(cnPoint.size() + ku);
    vector<float> knots_v(cnPoint[0].size() + kv);

    for (int i = 0; i < ku; ++i)
    {
        knots_u[i] = 0.0f;
        knots_u[knots_u.size() - 1 - i] = static_cast<float>(cnPoint.size() - ku + 1);
    }
    for (int i = ku; i < knots_u.size() - ku; ++i)
    {
        knots_u[i] = static_cast<float>(i - ku + 1);
    }

    for (int i = 0; i < kv; ++i)
    {
        knots_v[i] = 0.0f;
        knots_v[knots_v.size() - 1 - i] = static_cast<float>(cnPoint[0].size() - kv + 1);
    }
    for (int i = kv; i < knots_v.size() - kv; ++i)
    {
        knots_v[i] = static_cast<float>(i - kv + 1);
    }


    // 创建 B-spline 曲面对象
    bspSurface surface(cnPoint, knots_u, knots_v);

    // 生成曲面点和法线，用于绘制
    vector<Eigen::Vector3d> vertices;
    surface.getFittingSurface(vertices, 0.05); // 0.05为step

    auto end = std::chrono::high_resolution_clock::now();
    double duration = time_inc(end, start);
    std::cout << "Bspline-fitting time: " << duration << " milliseconds" << std::endl;  
    std::cout << "The height difference is: " << max_pt.z - min_pt.z << std::endl;

    // 提取控制点
    vector<Eigen::Vector3d> control_points;
    for (const auto& row : cnPoint)
    {
        for (const auto& point : row)
        {
            control_points.push_back(point);
        }
    }

    // 可视化拟合点和控制点
    visualizePointCloud(filtered_cloud, vertices, ground_cloud);

    return 0;
}
