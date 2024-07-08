#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <boost/filesystem.hpp>


std::string getFileNameWithoutExtension(const std::string& filePath) {
    // 找到最后一个路径分隔符的位置
    size_t lastSlash = filePath.find_last_of("/\\");
    std::string fileNameWithExtension = filePath.substr(lastSlash + 1);

    // 找到最后一个点的位置
    size_t lastDot = fileNameWithExtension.find_last_of(".");
    return fileNameWithExtension.substr(0, lastDot);
}


std::string getCurrentTimeString()
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
    return ss.str();
}


void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg, const std::string &window_name)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(window_name));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ori_color(cloud, 255, 255, 255); 
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> seg_color(cloud_seg, 255, 0, 0); 

    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, ori_color, "origi cloud");
    viewer->addPointCloud<pcl::PointXYZ>(cloud_seg, seg_color, "segment cloud");


    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "origi cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "segment cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


void visualizePointCloudSingle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg, const std::string &window_name)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(window_name));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> seg_color(cloud_seg, 255, 0, 0); 

    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_seg, seg_color, "segment cloud");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "segment cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


void visualizePointCloudSingleMLS(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_seg, const std::string &window_name)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(window_name));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> seg_color(cloud_seg, 255, 0, 0); 

    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointNormal>(cloud_seg, seg_color, "mls cloud");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "mls cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


void VoxelDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered) {


    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.3f, 0.3f, 0.3f);
    vg.filter(*voxel_filtered);

    ROS_INFO("the number of v-downsampled ptc is %ld", voxel_filtered->size());
//     // Create the filtering object
//     pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//     sor.setInputCloud (cloud);
//     sor.setLeafSize (0.01f, 0.01f, 0.01f);
//     sor.filter (*voxel_filtered);
}


int RANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud , pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg, float dis_thre) {

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // 可选的设置
    seg.setOptimizeCoefficients(true);
    // set the type of segment model
    // default iterations: 1000 for PCL   seg.setMaxIterations (100);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(dis_thre);

    // 设置输入点云
    seg.setInputCloud(input_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
        return -1;
    }

    ROS_INFO("the number of segmented ptc is %ld", inliers->indices.size());
    // // 提取平面内的点云

    pcl::ExtractIndices<pcl::PointXYZ> extract; 
    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_seg);
    return 0;

}


void PMF_Segment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered) {


    pcl::PointIndicesPtr ground(new pcl::PointIndices);
    // Create the filtering object
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud(cloud);
    pmf.setMaxWindowSize(10); // 最大窗口尺寸，对于较大地形可以色和之更大的最大窗口尺寸。一般最大窗口尺寸比初始窗口尺寸大一个数量级以上。
    pmf.setSlope(1.0f);
    pmf.setInitialDistance(0.5f);
    pmf.setMaxDistance(3.0f);
    pmf.extract(ground->indices);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(ground);
    extract.filter(*cloud_filtered);

    std::cerr << "Ground cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

}


void MLS_Surface(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_seg, pcl::PointCloud<pcl::PointNormal>::Ptr &mls_points, int poly_order, float search_r) {


    if (cloud_seg->empty()) {
        PCL_ERROR("Input cloud is empty, cannot apply MLS.\n");
        return;
    }
    // create a kd-tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    // pcl::PointCloud<pcl::PointNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    
    mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud (cloud_seg);
    mls.setPolynomialOrder (poly_order);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (search_r);

    // Reconstruct
    mls.process (*mls_points);  
    ROS_INFO("the number of smoothed ptc is %ld", mls_points->size());      
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_segmentation_example");
    ros::NodeHandle nh("~");

    std::string pcd_file_path;
    std::string output_folder;

    bool vis;
    nh.param<bool>("vis", vis, false);

    //获取参数，如果参数不存在则设置一个默认值
    if (!nh.getParam("/plane_model/pcd_file_path", pcd_file_path))
    {
        ROS_ERROR("Failed to get param 'pcd_file_path'");
        return -1;
    }

    // 创建点云对象的智能指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 读取PCD文件到点云对象中
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud) == -1)
    {
        ROS_ERROR("Couldn't read file %s", pcd_file_path.c_str());
        return -1;
    }

    std::string fileName = getFileNameWithoutExtension(pcd_file_path);


    ROS_INFO("the number before segmentation is %ld", cloud->size());


    // 获取输出文件夹路径，如果参数不存在则设置一个默认值
    if (!nh.getParam("/plane_model/output_folder", output_folder))
    {
        output_folder = ".";
        ROS_WARN("Output folder not specified. Using current directory.");
    }

    // 检查输出文件夹是否存在，不存在则创建
    // if (!std::filesystem::exists(output_folder))
    // {
    //     if (!std::filesystem::create_directories(output_folder))
    //     {
    //         ROS_ERROR("Failed to create output folder '%s'", output_folder.c_str());
    //         return -1;
    //     }
    // }

    if (!boost::filesystem::exists(output_folder))
    {
        if (!boost::filesystem::create_directories(output_folder))
        {
            ROS_ERROR("Failed to create output folder '%s'", output_folder.c_str());
            return -1;
        }
    }

    float dis_thre;
    nh.param<float>("dis_thre", dis_thre, 1);
    // ROS_INFO("the dis_thre is %f", dis_thre);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    VoxelDownSample(cloud, voxel_filter_cloud);


    std::string seg_type;
    nh.param<std::string>("seg_type", seg_type, "RANSAC");

    if (seg_type == "RANSAC")
    {
        RANSAC(voxel_filter_cloud , cloud_seg, dis_thre);
        ROS_INFO("The Segment Algrotihm is RANSAC and the number of Segmented is %ld", cloud_seg->size());
    } else if (seg_type == "PMF") {
        // 渐近形态滤波
        PMF_Segment(voxel_filter_cloud, cloud_seg);
        ROS_INFO("The Segment Algrotihm is PMF and the number of Segmented is %ld", cloud_seg->size());
    }

    // save segmented result obstacle_2407051534_h3_hill_tree_v_20c_seged
    // std::string output_pcd_file = output_folder + "/segmented_plane_" + getCurrentTimeString() + ".pcd";

    std::string output_pcd_file = output_folder + "/" + fileName + "_" +seg_type + "_seged" + ".pcd";   
    pcl::io::savePCDFileASCII(output_pcd_file, *cloud_seg);
    ROS_INFO("Saved segmented point cloud to %s", output_pcd_file.c_str());

    bool MLS;
    nh.param<bool>("MLS", MLS, false);

    if (MLS) {
        ROS_INFO("MLS OPEN");
        // moving least square

        int poly_order;
        float search_r;
        nh.param<int>("poly_order", poly_order, 2);
        nh.param<float>("search_r", search_r, 1);
        // ROS_INFO("%d, %f", poly_order, search_r);

        pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);
        MLS_Surface(cloud_seg, mls_points, poly_order, search_r);
        if (mls_points->empty()) {
            PCL_ERROR("MLS points is empty after MLS processing.\n");
            return (-1);
        }
        std::string out_mls = output_folder + "/" + fileName + "_" +seg_type+ "_mls.pcd";
        pcl::io::savePCDFileASCII(out_mls, *mls_points);
        ROS_INFO("Saved MLS point cloud to %s", out_mls.c_str());

        if (vis) {
            visualizePointCloudSingleMLS(mls_points, "Smooth Result");
        }


    }

    if (vis) {
        visualizePointCloudSingle(voxel_filter_cloud, "Voxel-Filter Result");
        visualizePointCloud(cloud, cloud_seg, "Original and Segmented Point Cloud Viewer");
        visualizePointCloudSingle(cloud_seg, "Segment Result");
    }

    return 0;
}
