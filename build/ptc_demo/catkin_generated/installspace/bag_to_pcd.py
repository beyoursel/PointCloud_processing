import rosbag
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import pcl


import rosbag
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import pcl


def read_bag_and_aggregate_pointcloud(bag_file, topic):
    bag = rosbag.Bag(bag_file)
    aggregated_points = []

    for topic, msg, t in bag.read_messages(topics=[topic]):
        # 将点云消息转换为点列表
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        for point in points:
            aggregated_points.append([point[0], point[1], point[2]])

    bag.close()
    
    return aggregated_points


def save_pointcloud_to_pcd(points, output_file):
    cloud = pcl.PointCloud()
    cloud.from_list(points)
    pcl.save(cloud, output_file)


if __name__ == "__main__":
    rospy.init_node('pointcloud_aggregator')

    bag_file = '/media/taole/HHD/Doc/daily_work/work_tg/ros_ws/raw_ptc_data/uav_radar_240709143739.bag'  # 替换为你的bag文件路径
    topic = '/mindcuise'  # 替换为你的点云话题

    print("Reading bag file and aggregating pointcloud data...")
    points = read_bag_and_aggregate_pointcloud(bag_file, topic)

    output_file = 'aggregated_pointcloud.pcd'
    print(f"Saving aggregated pointcloud to {output_file}...")
    save_pointcloud_to_pcd(points, output_file)

    print("Done!")
