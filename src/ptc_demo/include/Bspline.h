#ifndef BSPLINEPOINTCLOUD_H
#define BSPLINEPOINTCLOUD_H

#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>

using namespace std;

namespace BspSurfaceRestruct {

// 定义点的结构体
struct Point {
    double x, y, z;

    // 比较操作符，按照 x 坐标排序
    bool operator<(const Point& other) const {
        return x < other.x;
    }
};


struct Grid {
    std::vector<Point> points;
};


class bspSurface
{
public:
	bspSurface() {}

	bspSurface(pcl::PointCloud<pcl::PointXYZ>::Ptr raw_pointcloud, int k, 
				float grid_size);

	// constructor
	bspSurface(const bspSurface& surface);

	// 函数运算符重载
	bspSurface& operator=(const bspSurface& surface);

	// 根据参数u,v计算曲面上的坐标
	Eigen::Vector3d calPos(const float& u, const float& v);

	Eigen::Vector3d calPos(const vector<Eigen::Vector3d>& controlpoint, const vector<float>& knots, const float& t);


	static bool compareHeight(const Point& a, const Point& b);


	void getFittingSurface(vector<Eigen::Vector3d>& vertices, float step);
	
	vector<vector<Eigen::Vector3d>> getcontrolpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr ground_pointcloud);
	
	// 获得地面点	
	void ExtractGroudpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_cloud);

	void Getgroundpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud);

	void setknotvector(vector<vector<Eigen::Vector3d>> cnPoint, vector<float>& knots_u_b, vector<float>& knots_v_b);
	// KNN插值
	float interpolatePoint(const std::vector<Point>& points, double x, double y, int k);
	void StatisticalRemoveOutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud , pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
	void PassthroghFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud , pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
	void VoxelDownsample(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud , pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
	std::vector<Point> pclPointCloudToVector(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

private:
	int m_nu; // u向 0-nu
	int m_nv; // v向 0-nv
	int m_ku; // u向阶
	int m_kv; // v向阶
	float grid_width;
	float grid_height;
	pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points;
	vector<vector<Eigen::Vector3d>> m_cnPoint; //控制网格点坐标 （nu+1）x(nv+1)
	vector<float> m_knots_u; // u向节点向量 u_0, ..., u_(nu+ku)
	vector<float> m_knots_v; // v向节点向量 v_0, ..., v_(nv+kv)
};

}
#endif