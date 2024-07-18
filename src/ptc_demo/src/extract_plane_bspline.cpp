#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Bspline.h>
#include <pcl/common/common.h>
#include <chrono>
#include <queue>
#include <limits>
using namespace std;


// 定义点的结构体
struct Point {
    double x, y, z;

    // 比较操作符，按照 x 坐标排序
    bool operator<(const Point& other) const {
        return x > other.x;
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


void visualizePointCloud(
    const vector<Eigen::Vector3d>& vertices,
    const vector<Eigen::Vector3d>& control_points)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr controlCloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& vertex : vertices)
    {
        cloud->points.push_back(pcl::PointXYZ(vertex[0], vertex[1], vertex[2]));
    }

    for (const auto& point : control_points)
    {
        controlCloud->points.push_back(pcl::PointXYZ(point[0], point[1], point[2]));
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 0, 255, 0);
    viewer->addPointCloud(cloud, cloud_color_handler, "fitted_points");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> control_color_handler(controlCloud, 255, 0, 0);
    viewer->addPointCloud(controlCloud, control_color_handler, "control_points");
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
float interpolatePoint(const std::vector<Point>& points, double x, double y, int k = 2) {
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


double time_inc(std::chrono::high_resolution_clock::time_point &t_end,
                std::chrono::high_resolution_clock::time_point &t_begin) {
  
  return std::chrono::duration_cast<std::chrono::duration<double>>(t_end -t_begin).count() * 1000;
}


int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input_pcd_file>" << std::endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s \n", argv[1]);
        return -1;
    }



    // create controlPoints grid from pointcloud
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    float x_range = max_pt.x - min_pt.x;
    float y_range = max_pt.y - min_pt.y;

    float low_peak = max_pt.z;
    // float low_peak = getAverageHeight(cloud);
    float grid_width = 15.0f;
    float grid_height = 15.0f;

    int M = static_cast<int>(x_range / grid_width) + 1;
    int N = static_cast<int>(y_range / grid_height) + 1;    

    vector<vector<Eigen::Vector3d>> cnPoint(M, vector<Eigen::Vector3d>(N));

    double maxDouble = std::numeric_limits<double>::max();

    // create control points grid
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
            // cnPoint[i][j] = Eigen::Vector3d(min_pt.x + i * grid_width, min_pt.y + j * grid_height, low_peak);
            cnPoint[i][j](0) = min_pt.x + i * grid_width;
            cnPoint[i][j](1) = min_pt.y + j * grid_height;
            cnPoint[i][j](2) = maxDouble;
        }
    }

    for (const auto& point : cloud->points) {
        int x_ptc = static_cast<int>(point.x);
        int y_ptc = static_cast<int>(point.y);
        cnPoint[(x_ptc - min_pt.x) / grid_width][(y_ptc - min_pt.y) / grid_height](2) = point.z;
    }
    

  
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
            std::cout << cnPoint[i][j] << std::endl;
        }
    }

    int countx = 1;
    std::vector<Point> ptc = pclPointCloudToVector(cloud);
    // // 插值填充空栅格
    for (int i = 0; i < M; ++i) {
        for (int j = 0; j < N; ++j) {
            if (cnPoint[i][j](2) == maxDouble) {
                double x_em = min_pt.x + i * grid_width;
                double y_em = min_pt.y + j * grid_height;
                cnPoint[i][j](2) = interpolatePoint(ptc, x_em, y_em);
                // std::cout << countx << std::endl;
                // countx++;
            }
        }
    }

  
    // for (size_t i = 0; i < M; i++) {
    //     for (size_t j = 0; j < N; j++) {
    //         std::cout << cnPoint[i][j] << std::endl;
    //     }
    // }

    // 动态调整控制点高度
    // for (size_t i = 0; i < M; i++) {
    //     for (size_t j = 0; j < N; j++) {
    //         double avg_height = cnPoint[i][j][2]; // 当前高度
    //         int count = 1;

    //         for (int di = -1; di <= 1; ++di) {
    //             for (int dj = -1; dj <= 1; ++dj) {
    //                 if (di == 0 && dj == 0) continue; // skip itself
    //                 int ni = i + di;
    //                 int nj = j + dj;
    //                 if (ni >= 0 && ni < M && nj >= 0 && nj < N) {
    //                     avg_height += cnPoint[ni][nj][2]; // 累加相邻高度
    //                     count++;
    //                 }
    //             }
    //         }

    //         cnPoint[i][j][2] = avg_height / count; // 更新高度为平均值
    //     }
    // }



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

    // for (auto& u_v: knots_u) {
    //     std::cout << u_v << std::endl;
    // }


    auto start = std::chrono::high_resolution_clock::now();

    // 创建 B-spline 曲面对象
    bspSurface surface(cnPoint, knots_u, knots_v);

    // 生成曲面点和法线，用于绘制
    vector<Eigen::Vector3d> vertices;
    // vector<Eigen::Vector3d> vertices, normals;
    // vector<unsigned short> edge_indices, face_indices;
    surface.getFittingSurface(vertices, 0.05); // 0.05为step

    auto end = std::chrono::high_resolution_clock::now();
    double duration = time_inc(end, start);
    std::cout << "Bspline-fitting time: " << duration << " milliseconds" << std::endl;  


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
    visualizePointCloud(vertices, control_points);

    // 保存曲面点为PCD文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr surfaceCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& vertex : vertices)
    {
        surfaceCloud->points.push_back(pcl::PointXYZ(vertex[0], vertex[1], vertex[2]));
    }
    surfaceCloud->width = surfaceCloud->points.size();
    surfaceCloud->height = 1; // Unordered point cloud
    surfaceCloud->is_dense = true;


    // 保存controlpoint为PCD文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr controlCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& ctrl_point : control_points)
    {
        controlCloud->points.push_back(pcl::PointXYZ(ctrl_point[0], ctrl_point[1], ctrl_point[2]));
    }
    controlCloud->width = controlCloud->points.size();
    controlCloud->height = 1; // Unordered point cloud
    controlCloud->is_dense = true;

    pcl::io::savePCDFile("control_surface.pcd", *controlCloud);
    pcl::io::savePCDFile("fitted_surface.pcd", *surfaceCloud);

    return 0;
}
