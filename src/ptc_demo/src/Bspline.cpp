#include "Bspline.h"
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <queue>
#include <limits>
#include <memory>
#include <boost/make_shared.hpp>

using namespace std;

namespace BspSurfaceRestruct {


bool bspSurface::compareHeight(const Point& a, const Point& b) {
    return a.z < b.z;
}


// 将pcl::PointCloud<pcl::PointXYZ>::Ptr转换为std::vector<Point>
std::vector<Point> bspSurface::pclPointCloudToVector(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
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


bspSurface::bspSurface(pcl::PointCloud<pcl::PointXYZ>::Ptr raw_pointcloud, int k, float grid_size) {

    m_ku = k;
    m_kv = k;
    grid_width = grid_size;
    grid_height = grid_size;
	// pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZ>);
    ground_points = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    ExtractGroudpoint(raw_pointcloud, ground_points);
    // std::cout << ground_points->points.size() << std::endl;
    m_cnPoint = getcontrolpoint(ground_points);
    setknotvector(m_cnPoint, m_knots_u, m_knots_v);
    m_nu = m_cnPoint.size() - 1;
    m_nv = m_cnPoint[0].size() - 1;

}

// constructor
bspSurface::bspSurface(const bspSurface& surface) {
    m_cnPoint = surface.m_cnPoint;
    m_knots_u = surface.m_knots_u;
    m_knots_v = surface.m_knots_v;

    m_nu = surface.m_nu;
    m_nv = surface.m_nv;
    m_ku = surface.m_kv;
    m_kv = surface.m_kv; 
}

// 函数运算符重载
bspSurface& bspSurface::operator=(const bspSurface& surface) {
    m_cnPoint = surface.m_cnPoint;
    m_knots_u = surface.m_knots_u;
    m_knots_v = surface.m_knots_v;

    m_nu = surface.m_nu;
    m_nv = surface.m_nv;
    m_ku = surface.m_ku;
    m_kv = surface.m_kv;
    return *this;
}

// 根据参数u,v计算曲面上的坐标
Eigen::Vector3d bspSurface::calPos(const float& u, const float& v) {

    vector<Eigen::Vector3d> v_constant(m_nu + 1);
    for (int i = 0; i < v_constant.size(); ++i)
    {
        v_constant[i] = calPos(m_cnPoint[i], m_knots_v, v);
    }
    return calPos(v_constant, m_knots_u, u);
}


Eigen::Vector3d bspSurface::calPos(const vector<Eigen::Vector3d>& controlpoint, const vector<float>& knots, const float& t)
{
    int n = controlpoint.size() - 1;
    int k = knots.size() - controlpoint.size(); // 阶数
    int L = 0;
    // 计算t所处的区间[t_L, t_(L+1)], t只在[knots[k-1], knots[n+1]]中有效
    if (t >= knots[n+1])
    {
        L = n;
    } else if (t <= knots[k-1])
    {
        L = k - 1;
    }
    else
    {
        for (int i = k - 1; i <= n + 1; ++i)
        {
            if (t >= knots[i] && t<knots[i+1])
            {
                L = i;
                break;
            }
        }
    }

    if (L >= n + 1) L = n;

    vector<Eigen::Vector3d> temp(k);
    for (int i = 0; i < k; ++i) {
        temp[i] = controlpoint[i + L - k + 1];
    }

    //de-BoorCox算法
    for (int r = 1; r <= k - 1; ++r)
    {
        for (int i = 0; i <= k - r - 1; ++i)
        {
            int index = L - k + 1 + r;
            double factor = 0;
            if (knots[index + i + k - r] != knots[index + i])
            {
                factor = (t - knots[index + i]) / (knots[index + i + k - r] - knots[index + i]);
            }
            temp[i] = factor*temp[i + 1] + (1 - factor)*temp[i];

        }
    }
    return temp[0];

}


void bspSurface::getFittingSurface(
    vector<Eigen::Vector3d>& vertices, float step)
{

    int m = static_cast<int>((m_knots_u[m_nu + 1] - m_knots_u[m_ku - 1]) / step);
    int n = static_cast<int>((m_knots_v[m_nv + 1] - m_knots_v[m_kv - 1]) / step);

    for (int i = 0; i <= m; ++i)
    {
        for (int j = 0; j <= n; ++j)
        {
            float u = 0, v = 0;
            if (i == m)
            {
                u = m_knots_u[m_nu + 1];
                v = m_knots_v[m_kv - 1] + j*step;
                
            }
            else if (j == n)
            {
                u = m_knots_u[m_ku - 1] + i*step;
                v = m_knots_v[m_nv + 1];
                
            }
            else
            {
                u = m_knots_u[m_ku - 1] + i*step;
                v = m_knots_v[m_kv - 1] + j*step;
            }
            
            Eigen::Vector3d temp = calPos(u, v);
            vertices.push_back(temp);
        }
    }
}


vector<vector<Eigen::Vector3d>> bspSurface::getcontrolpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr ground_pointcloud)
{

    pcl::PointXYZ min_pt_g, max_pt_g;
    pcl::getMinMax3D(*ground_pointcloud, min_pt_g, max_pt_g);

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

    for (const auto& point : ground_pointcloud->points) {
        float x_ptc = static_cast<float>(point.x);
        float y_ptc = static_cast<float>(point.y);
        cnPoint[(x_ptc - min_pt_g.x) / grid_width][(y_ptc - min_pt_g.y) / grid_height](2) = point.z;
    }

    std::vector<Point> ptc = pclPointCloudToVector(ground_pointcloud);
    // // 插值填充空栅格
    for (int i = 0; i < M_g; ++i) {
        for (int j = 0; j < N_g; ++j) {
            if (cnPoint[i][j](2) == maxDouble) {
                double x_em = min_pt_g.x + i * grid_width;
                double y_em = min_pt_g.y + j * grid_height;
                cnPoint[i][j](2) = interpolatePoint(ptc, x_em, y_em, 3);
            }
        }
    }


    return cnPoint;
}


void bspSurface::setknotvector(vector<vector<Eigen::Vector3d>> cnPoint, vector<float>& knots_u_b, vector<float>& knots_v_b) {

    // 设置均匀节点向量
    vector<float> knots_u(cnPoint.size() + m_ku);
    vector<float> knots_v(cnPoint[0].size() + m_kv);

    for (int i = 0; i < m_ku; ++i)
    {
        knots_u[i] = 0.0f;
        knots_u[knots_u.size() - 1 - i] = static_cast<float>(cnPoint.size() - m_ku + 1);
    }
    for (int i = m_ku; i < knots_u.size() - m_ku; ++i)
    {
        knots_u[i] = static_cast<float>(i - m_ku + 1);
    }

    for (int i = 0; i < m_kv; ++i)
    {
        knots_v[i] = 0.0f;
        knots_v[knots_v.size() - 1 - i] = static_cast<float>(cnPoint[0].size() - m_kv + 1);
    }
    for (int i = m_kv; i < knots_v.size() - m_kv; ++i)
    {
        knots_v[i] = static_cast<float>(i - m_kv + 1);
    }

    knots_u_b = knots_u;
    knots_v_b = knots_v;
}


// 插值函数：使用KNN（k近邻）算法来填充空栅格
float bspSurface::interpolatePoint(const std::vector<Point>& points, double x, double y, int k) {
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


void bspSurface::StatisticalRemoveOutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud , pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered) {
      // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (input_cloud);
    sor.setMeanK (10);
    sor.setStddevMulThresh (2.0);
    sor.filter (*cloud_filtered);
}


void bspSurface::PassthroghFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud , pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered) {
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


void bspSurface::VoxelDownsample(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud , pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;

    sor.setInputCloud(input_cloud);
    sor.setLeafSize(0.5f, 0.5f, 0.5f);  // voxel_size 
    sor.filter(*cloud_filtered);
}


void bspSurface::ExtractGroudpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_cloud)
{

    std::cout << "the number of raw ptc is: " << cloud->points.size() << std::endl;
    VoxelDownsample(cloud, cloud);
    std::cout << "the number of downsample ptc is: " << cloud->points.size() << std::endl;

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt); //


    float x_range = max_pt.x - min_pt.x;
    float y_range = max_pt.y - min_pt.y;
    // grid_size
    float grid_width = 10.0f;
    float grid_height = 10.0f;

    int M = static_cast<int>(x_range / grid_width) + 2;
    int N = static_cast<int>(y_range / grid_height) + 2;
     
    // std::cout << "M: " << M << " N: " << N << std::endl;
    std::vector<std::vector<Grid>> grids(M, std::vector<Grid>(N));

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& point = cloud->points[i];
        int grid_x = static_cast<int>((point.x - min_pt.x) / grid_width);
        int grid_y = static_cast<int>((point.y - min_pt.y) / grid_height);

        if (grid_x >= 0 && grid_x < M && grid_y >= 0 && grid_y < N) {

            grids[grid_x][grid_y].points.push_back({point.x, point.y, point.z});

        }
    }

    for (int i = 0; i < M; ++i) {
        for (int j = 0; j < N; ++j) {
            auto& grid_points = grids[i][j].points;
            if (!grid_points.empty()) {


                std::sort(grid_points.begin(), grid_points.end(), compareHeight);

                size_t retain_count = static_cast<size_t>(grid_points.size() * 0.1);
                if (retain_count < 1) {
                    retain_count = 1;
                }
                // std::cout << retain_count << std::endl;
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
    // std::cout << "ground points size is: " << ground_cloud->points.size() << std::endl;
    StatisticalRemoveOutlier(ground_cloud, ground_cloud);    
    std::cout << "ground points size is: " << ground_cloud->points.size() << std::endl;
}


void bspSurface::Getgroundpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud) {

    ground_cloud->points = ground_points->points;

}

}