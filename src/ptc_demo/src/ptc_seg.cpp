#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char** argv) {
    // ros::init(argc, argv, "pointcloud_reader");
    // ros::NodeHandle nh;

    // // 创建一个发布器来发布点云数据
    // ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);

    // 读取PCD文件
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/media/taole/ssd1/work_top&gun/ros_ws/data_for_seg/obstacle_2407031653.pcd", cloud) == -1) {
        ROS_ERROR("Couldn't read file");
        return -1;
    }


    // 将PCL点云转换为ROS消息
    // sensor_msgs::PointCloud2 output;
    // pcl::toROSMsg(cloud, output);
    // output.header.frame_id = "map";

    // ros::Rate loop_rate(1);
    // while (ros::ok()) {
    //     pcl_pub.publish(output);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    
    return 0;
}
