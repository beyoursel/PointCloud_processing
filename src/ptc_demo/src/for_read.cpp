#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <limits>
#include <cmath>
#include <algorithm>
#include <queue>

// 定义一个简单的Point结构体
struct Point {
    double x, y, z;
};

// 比较函数，用于点排序
bool comparePoints(const Point& p1, const Point& p2) {
    return (p1.x < p2.x) || (p1.x == p2.x && p1.y < p2.y);
}

// 计算叉积
double cross(const Point& O, const Point& A, const Point& B) {
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

// 计算凸包
std::vector<Point> convexHull(std::vector<Point>& points) {
    std::sort(points.begin(), points.end(), comparePoints);
    std::vector<Point> hull;

    // 构建下半部分凸包
    for (const auto& p : points) {
        while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.back(), p) <= 0) {
            hull.pop_back();
        }
        hull.push_back(p);
    }

    // 构建上半部分凸包
    size_t lowerHullSize = hull.size();
    for (int i = points.size() - 2; i >= 0; --i) {
        while (hull.size() > lowerHullSize && cross(hull[hull.size() - 2], hull.back(), points[i]) <= 0) {
            hull.pop_back();
        }
        hull.push_back(points[i]);
    }

    hull.pop_back(); // 移除重复的最后一个点
    return hull;
}

// 计算两点之间的距离
double distance(const Point& A, const Point& B) {
    return std::sqrt((A.x - B.x) * (A.x - B.x) + (A.y - B.y) * (A.y - B.y));
}

// 计算两点之间的角度
double angle(const Point& A, const Point& B) {
    return std::atan2(B.y - A.y, B.x - A.x);
}

// 计算最小包络矩形
void computeMinBoundingBox(const std::vector<Point>& points, Point& min_pt, Point& max_pt) {
    std::vector<Point> hull = convexHull(const_cast<std::vector<Point>&>(points));
    size_t n = hull.size();

    double minArea = std::numeric_limits<double>::max();
    Point best_min_pt, best_max_pt;

    for (size_t i = 0; i < n; ++i) {
        const Point& A = hull[i];
        const Point& B = hull[(i + 1) % n];

        double theta = angle(A, B);
        double cos_theta = std::cos(theta);
        double sin_theta = std::sin(theta);

        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::min();
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::min();

        for (const auto& p : hull) {
            double x = p.x * cos_theta + p.y * sin_theta;
            double y = -p.x * sin_theta + p.y * cos_theta;

            if (x < min_x) min_x = x;
            if (x > max_x) max_x = x;
            if (y < min_y) min_y = y;
            if (y > max_y) max_y = y;
        }

        double area = (max_x - min_x) * (max_y - min_y);
        if (area < minArea) {
            minArea = area;

            best_min_pt = {min_x * cos_theta - min_y * sin_theta, min_x * sin_theta + min_y * cos_theta};
            best_max_pt = {max_x * cos_theta - max_y * sin_theta, max_x * sin_theta + max_y * cos_theta};
        }
    }

    min_pt = best_min_pt;
    max_pt = best_max_pt;
}

// 插值函数：使用KNN（k近邻）算法来填充空栅格
Eigen::Vector3d interpolatePoint(const std::vector<Point>& points, double x, double y, int k = 3) {
    std::priority_queue<std::pair<double, Point>> pq;

    for (const auto& pt : points) {
        double dist = std::sqrt((pt.x - x) * (pt.x - x) + (pt.y - y) * (pt.y - y));
        pq.push(std::make_pair(dist, pt));
        if (pq.size() > k) {
            pq.pop();
        }
    }

    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    int count = 0;
    while (!pq.empty()) {
        auto top = pq.top();
        pq.pop();
        sum_x += top.second.x;
        sum_y += top.second.y;
        sum_z += top.second.z;
        count++;
    }

    return Eigen::Vector3d(sum_x / count, sum_y / count, sum_z / count);
}

void fillPointCloudToGrid(const std::vector<Point>& points, std::vector<std::vector<Eigen::Vector3d>>& grid,
                          const Point& min_pt, const Point& max_pt, int rows, int cols) {
    double x_step = (max_pt.x - min_pt.x) / (cols - 1);
    double y_step = (max_pt.y - min_pt.y) / (rows - 1);

    // 初始化栅格
    grid.resize(rows, std::vector<Eigen::Vector3d>(cols, Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),
                                                                          std::numeric_limits<double>::quiet_NaN(),
                                                                          std::numeric_limits<double>::quiet_NaN())));

    // 将点分配到网格中
    for (const auto& pt : points) {
        int row = static_cast<int>((pt.y - min_pt.y) / y_step);
        int col = static_cast<int>((pt.x - min_pt.x) / x_step);

        if (row >= 0 && row < rows && col >= 0 && col < cols) {
            grid[row][col] = Eigen::Vector3d(pt.x, pt.y, pt.z);
        }
    }

    // 插值填充空栅格
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (std::isnan(grid[i][j].z())) {
                double x = min_pt.x + j * x_step;
                double y = min_pt.y + i * y_step;
                grid[i][j] = interpolatePoint(points, x, y);
            }
        }
    }
}

int main() {
    // 示例点云数据
    std::vector<Point> points = {
        {0.1, 0.1, 1.0}, {0.4, 0.1, 2.0}, {0.7, 0.1, 3.0},
        {0.1, 0.4, 4.0}, {0.4, 0.4, 5.0}, {0.7, 0.4, 6.0},
        {0.1, 0.7, 7.0}, {0.4, 0.7, 8.0}, {0.7, 0.7, 9.0}
    };

    // 计算最小包络矩形
    Point min_pt, max_pt;
    computeMinBoundingBox(points, min_pt, max_pt);

    // 栅格化
    int rows = 3, cols = 3;
    std::vector<std::vector<Eigen::Vector3d>> grid;
    fillPointCloudToGrid(points, grid, min_pt, max_pt, rows, cols);

    // 输出结果
    std::cout << "Grid:" << std::endl;
    for (const auto& row : grid) {
        for (const auto& cell : row) {
            std::cout << "(" << cell.x() << ", " << cell.y() << ", " << cell.z() << ") ";
        }
        std::cout << std::endl;
    }

    return 0;
}
