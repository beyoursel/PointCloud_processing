#include <iostream>
#include <fstream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

struct PointXYZ {
    float x, y, z;
};

bool loadBinFile(const std::string &filename, std::vector<PointXYZ> &points) {
    std::ifstream input(filename, std::ios::binary);
    if (!input) {
        std::cerr << "Cannot open file: " << filename << std::endl;
        return false;
    }
    
    PointXYZ point;
    while (input.read(reinterpret_cast<char*>(&point), sizeof(PointXYZ))) {
        points.push_back(point);
    }

    input.close();
    return true;
}

bool savePCDFile(const std::string &filename, const std::vector<PointXYZ> &points) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (const auto &point : points) {
        cloud.push_back(pcl::PointXYZ(point.x, point.y, point.z));
    }

    if (pcl::io::savePCDFileASCII(filename, cloud) == -1) {
        std::cerr << "Failed to save PCD file: " << filename << std::endl;
        return false;
    }

    return true;
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input_bin_file> <output_pcd_file>" << std::endl;
        return -1;
    }

    std::string input_filename = argv[1];
    std::string output_filename = argv[2];

    std::vector<PointXYZ> points;
    if (!loadBinFile(input_filename, points)) {
        return -1;
    }

    if (!savePCDFile(output_filename, points)) {
        return -1;
    }

    std::cout << "Successfully converted " << input_filename << " to " << output_filename << std::endl;
    return 0;
}
