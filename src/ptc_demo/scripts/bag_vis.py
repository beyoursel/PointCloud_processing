"""
  bag包数据可视化
"""

# Radar Vis - Show radar detections.
# Copyright 2008 TopXGun Robotics.  All rights reserved.
#
# Su Hua  hsu@topxgun.com
# version :
#
#  0.1 支持Rosbag MindCruise避障雷达点云解析。
import csv
import os
import sys
sys.path.append("../../../../devel/lib/python3/dist-packages")

import struct
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header,Float32
from mapping_msgs.msg import PtzAngle
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from datetime import datetime

from math import sqrt
import math
import rosbag
import rospy
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

USAGE = "Parse and Show 4D Radar Bag File.\n\
    Usage: python3 bag_vis.py -r radar_xxx.bag [-s]\n\n\
    D - move forward a frame.\n\
    A - move backward a frame.\n\
    N - only show now frame.\n\
    R - only show recent frames.\n\
    If you want to change your data file to bag, use:\n\
    python3 bag_vis.py radar_ptc_xxx.data or \n\
    python3 bag_vis.py radar_xxx_path."

# radar type
RADAR_TYPE_FRONT = 0
RADAR_TYPE_REAR = 1
RADAR_TYPE_TERRAIN = 2

# filter param
RCS_THRESHOLD = 45
CONFIDENCE_LIMIT = 30
YAW_RATE_LIMIT = 100
POSE_RATE_LIMIT = 100
TO_LAND_LIMIT = 1.2
SHOW_RECENT_NUMBER = 1500

# vis control
TRACE_COLOR = [1, 0.706, 0]  # yellow
NOW_FRAME_COLOR = [0.5, 0, 0.5]  # purple
GROUND_COLOR = [0.2, 0.2, 0.2]  # gray

# TF: world:(enu), body:(front right down), ob radar:(front left up), terrain radar:(left front down)
#        z  y                  x                     x  z                          z
#        | /                  /                       \ |                           \
#        |/_ _ x             /_ _ _ y             y _ _\|                            \_ _ _ x
#                            |                                                       |
#                            |                                                       |
#                            z                                                       y
rot_enu_ned = R.from_euler("xyz", (180, 0, 90), degrees=True).as_matrix()
rot_body_fradar = R.from_euler("x", 180, degrees=True).as_matrix()
rot_body_rradar = R.from_euler("y", 180, degrees=True).as_matrix()
rot_body_tradar = R.from_euler("xyz", (0, 0, 0), degrees=True).as_matrix()
t_body_fradar = np.array([0.24, 0.05, 0.18])
# t_body_fradar = np.array([0, 0, 0])
INSTALL_VERTICAL_4D_ANGLE = 10


class PointXYZINormal:
    def __init__(self, x, y, z, intensity, normal_x, normal_y, normal_z, curvature):
        self.x = x
        self.y = y
        self.z = z
        self.intensity = intensity
        self.normal_x = normal_x
        self.normal_y = normal_y
        self.normal_z = normal_z
        self.curvature = curvature

# 将点坐标转换为字节序列。


def point_to_bytes(point):
    # 将点的属性转换为字节序列
    byte_data = struct.pack(f"3f",
                            point.x, point.y, point.z)
    padding = b'\x00'*4
    line2 = struct.pack(f"3f",
                        point.normal_x, point.normal_y, point.normal_z)
    padding2 = b'\x00'*4
    line3 = struct.pack(f"2f",
                        point.intensity, point.curvature)
    padding3 = b'\x00'*8
    # 将字节序列拼接成一个大的字节序列
    return byte_data + padding + line2 + padding2 + line3 + padding3

# 解析ROSbag文件


def parse_ros_bag(file_path, vertical_install=False):

    #    雷达安装信息
    if vertical_install:
        install_angle = INSTALL_VERTICAL_4D_ANGLE
        rot_radar_install = R.from_euler(
            "xyz", (90, install_angle, 0), degrees=True).as_matrix()
    else:
        install_angle = INSTALL_VERTICAL_4D_ANGLE
        rot_radar_install = R.from_euler(
            "y", install_angle * -1.0, degrees=True).as_matrix()
    target_frames = []

    #   topic：'radar_points', 'uav_pos','uav_vel','ptz_angle'
    topic_filter = ["uav_pos", 'uav_vel', "radar_points"]
    with rosbag.Bag(file_path, "r") as bag:
        for topic, msg, t in bag.read_messages(topic_filter):

            if topic == "uav_pos":
                position = np.array(
                    [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
                pose_quat = np.array(
                    [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
                rot_enu_radar = R.from_quat(
                    pose_quat).as_matrix() @ rot_body_fradar
            elif topic == "uav_vel":
                vel = np.array(
                    [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
                pose_rate = np.array(
                    [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])

            elif topic == "radar_points":

                targets = []
                gen = pc2.read_points(
                    msg, skip_nans=True, field_names=("x", "y", "z"))
                for p in gen:
                    target = [float(p[0]), float(p[1]), float(p[2])]
                    targets.append(rot_enu_radar @ rot_radar_install @
                                   np.array(target) + position)

                if max([abs(rate) for rate in pose_rate[0:2]]) > POSE_RATE_LIMIT:
                    targets = []
                    print("pose rate too high")

                if abs(pose_rate[2]) > YAW_RATE_LIMIT:
                    targets = []
                    print("yaw rate too high")

                target_frames.append(
                    [position, pose_quat, targets, pose_rate, vel])

    return target_frames


def string_to_time(time_string):
    # 将字符串转换为datetime对象
    # dt = datetime.strptime(time_string, '%Y-%m-%d %H:%M:%S.%d')

    dt = datetime.strptime(time_string, '%Y-%m-%d %H:%M:%S.%f')
    # 将datetime对象转换为Unix时间戳
    timestamp = dt.timestamp()

    # 将Unix时间戳转换为rospy.Time
    ros_time = rospy.Time(timestamp)

    return ros_time


def create_pose_stamped_msg(position, orientation, frame_id, timestamp=None):
    # 创建PoseStamped消息实例
    pose_stamped_msg = PoseStamped()

    # 设置header中的frame_id和timestamp
    pose_stamped_msg.header.frame_id = frame_id
    if timestamp is None:
        pose_stamped_msg.header.stamp = rospy.Time.now()
    else:
        pose_stamped_msg.header.stamp = timestamp

    # 设置位姿信息
    pose_stamped_msg.pose.position.x = position[0]
    pose_stamped_msg.pose.position.y = position[1]
    pose_stamped_msg.pose.position.z = position[2]

    # 设置四元数方向信息
    pose_stamped_msg.pose.orientation.x = orientation[0]
    pose_stamped_msg.pose.orientation.y = orientation[1]
    pose_stamped_msg.pose.orientation.z = orientation[2]
    pose_stamped_msg.pose.orientation.w = orientation[3]

    return pose_stamped_msg


def create_ptz_angle_msg(angle, frame_id, timestamp=None):
    # 创建PtzAngle消息实例
    ptz_angle_msg = PtzAngle()
    # 设置header中的frame_id和timestamp
    
    ptz_angle_msg.header.frame_id = frame_id
    if timestamp is None:
        ptz_angle_msg.header.stamp = rospy.Time.now()
    else:
        ptz_angle_msg.header.stamp = timestamp

    # 设置角度信息
    ptz_angle_msg.ptz_angle = Float32(angle)
    
    return ptz_angle_msg


def create_twist_stamped_msg(linear_velocity, angular_velocity, frame_id, timestamp=None):
    # 创建TwistStamped消息实例
    twist_stamped_msg = TwistStamped()

    # 设置header中的frame_id和timestamp
    twist_stamped_msg.header.frame_id = frame_id
    if timestamp is None:
        twist_stamped_msg.header.stamp = rospy.Time.now()
    else:
        twist_stamped_msg.header.stamp = timestamp

    # 设置Twist信息
    twist_stamped_msg.twist.linear.x = linear_velocity[0]
    twist_stamped_msg.twist.linear.y = linear_velocity[1]
    twist_stamped_msg.twist.linear.z = linear_velocity[2]

    twist_stamped_msg.twist.angular.x = angular_velocity[0]
    twist_stamped_msg.twist.angular.y = angular_velocity[1]
    twist_stamped_msg.twist.angular.z = angular_velocity[2]

    return twist_stamped_msg


def create_point_cloud_normals_msg(points, frame_id='mindcruise', timestamp=None):

    # 点云数据的字段定义
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('intensity', 32, PointField.FLOAT32, 1),
              PointField('normal_x', 16, PointField.FLOAT32, 1),
              PointField('normal_y', 20, PointField.FLOAT32, 1),
              PointField('normal_z', 24, PointField.FLOAT32, 1),
              PointField('curvature', 36, PointField.FLOAT32, 1)]

    # 创建头部信息
    header = Header()
    header.frame_id = frame_id
    if timestamp is None:
        # 如果没有提供时间戳，使用当前时间
        header.header.stamp = rospy.Time.now()
    else:
        header.stamp = timestamp

    # 点云数据的布局

    point_step = 48   # 每个点的字节数
    row_step = point_step * points.shape[0]  # 每行的字节数

    # 将点云数据转换为字节序列
    data = b''.join(point_to_bytes(p) for p in points)

    # 创建PointCloud2消息
    cloud_msg = PointCloud2(
        header=header,
        height=1,  # 默认为1，表示点云是二维的
        width=points.shape[0],
        is_dense=True,  # 可能存在NaN或无穷大值
        is_bigendian=False,  # 小端模式
        fields=fields,
        point_step=point_step,
        row_step=row_step,
        data=data
    )
    return cloud_msg

# 该函数将指定文件路径中的数据转换为bag对象


def data_to_bag(file_path):
   # version 2
    # Log format: date time log_type coord_type radar_type uav_x uav_y uav_z terrian_height vel_e vel_n vel_u roll pitch yaw roll_rate pitch_rate yaw_rate
    # install_angle [x y z vel rcs confidence beam ...]
    # coord_type: 0-极坐标 1-笛卡尔

    # version 200
    # Log format: date time log_type coord_type radar_type uav_x uav_y uav_z terrian_height vel_e vel_n vel_u roll pitch yaw roll_rate pitch_rate yaw_rate
    # install_angle [x y z vel rcs confidence beam id flags snr doppler_confidence frameext_counter...]
    # coord_type: 0-极坐标 1-笛卡尔

    segment_count = 0
    target_frames = []
    version_num = 0

    real_path = os.path.dirname(file_path)
    filename, file_extension = os.path.splitext(os.path.basename(file_path))
    bag = rosbag.Bag(real_path + "/"+filename + ".bag", 'w')

    with open(file_path, "r") as f:
        reader = csv.reader(f, delimiter=" ", skipinitialspace=True)
        for row in reader:
            if len(row) <= 18:
                continue

            targets = []
            time_str = row[0]+' ' + row[1]
            coord_type = int(row[3])
            radar_type = int(row[4])
            version_num = int(row[2])
            position = np.array([float(row[5]), float(row[6]), float(row[7])])
            vel = np.array([float(row[9]), float(row[10]), float(row[11])])
            pose = np.array([float(row[12]), float(row[13]), float(row[14])])
            pose_rate = np.array(
                [float(row[15]), float(row[16]), float(row[17])])
            install_angle = float(row[18])
            if (version_num == 200):
                segment_count = 12
            else:
                segment_count = 7

            if len(target_frames) > 5:
                # if False:
                position_ = target_frames[-2][1]
                pose_ = target_frames[-2][2]
            else:
                position_ = position
                pose_ = pose
            target_frames.append([time_str, position, pose])
            # 时间戳

            ros_time = string_to_time(time_str)

            # 封装位姿消息
            rot_ned_body = R.from_euler("xyz", tuple(pose_), True).as_matrix()
            rot_enu_body = rot_enu_ned @ rot_ned_body
            quaternion = R.from_matrix(rot_enu_body).as_quat()
            pose_msg = create_pose_stamped_msg(
                position_, quaternion, "enu", ros_time)
            bag.write("uav_pos", pose_msg, ros_time)
            # 封装安装角度
            angle_msg = create_ptz_angle_msg(0.0, "body", ros_time)
            bag.write("ptz_angle", angle_msg, ros_time)
            # 封装速度消息
            twist_stamp_msg = create_twist_stamped_msg(
                vel, pose_rate, "body", ros_time)
            bag.write("uav_vel", twist_stamp_msg, ros_time)
            # 封装点云消息
            for idx in range(19, len(row) - 1, segment_count):
                snr = float(row[idx + 3])
                rcs = float(row[idx + 4])
                confidence = int(row[idx + 5])
                if segment_count > 7:
                    vel_radar = float(row[idx + 9])
                else:
                    vel_radar = 0

                if abs(confidence) < CONFIDENCE_LIMIT:
                    continue

                if coord_type == 0:
                    d, azimuth, elevation = float(row[idx]), float(
                        row[idx+1]), float(row[idx + 2])
                    rot_yaw = R.from_euler(
                        "z", azimuth, True).as_matrix()
                    rot_pitch = R.from_euler(
                        "y", elevation, True).as_matrix()
                    [x, y, z] = rot_yaw@rot_pitch@[d, 0, 0]
                    i = 0
                    normal_x = vel_radar
                    normal_y = rcs
                    normal_z = snr
                    target = PointXYZINormal(
                        x, y, z, i, normal_x, normal_y, normal_z, 0)
                    targets.append(target)
            cloud_msgs = create_point_cloud_normals_msg(
                np.array(targets), "mindcruise", ros_time)

            bag.write("radar_points", cloud_msgs, ros_time)

    bag.close()
    return 0

def list_files(directory):
    file_paths = []
    for filename in os.listdir(directory):
        filepath = os.path.join(directory, filename)
        if os.path.isfile(filepath):
           file_paths.append(filepath)
    return file_paths


def data_path_to_bag(path):
    if os.path.isfile(path):
        exit(0)
    else:
        # 调用函数并传入你的目录路径
        file_paths = list_files(path)
        
    for file_name in file_paths:
        print(file_name)
        if (file_name.find("bag")) != -1:
            continue
        elif (file_name.find("radar_ptc")) != -1:
            data_to_bag(file_name)
    return 0

def draw_point_cloud(frames):
    vis = o3d.visualization.VisualizerWithKeyCallback()
    all_pcd = o3d.geometry.PointCloud()
    now_pcd = o3d.geometry.PointCloud()
    uav_trace = o3d.geometry.PointCloud()
    body_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.8)
    rot_last = R.from_euler("xyz", [0, 0, 0], degrees=True).as_matrix()
    ptc = []
    trace = []
    ptc_idx = 0
    frame_idx = 0
    show_ground = True
    show_recent = False
    show_now = False
    playing = False

    # draw ground plane
    points = []
    lines = []
    for i in range(0, 201, 1):
        points += [[100, i - 100, 0], [-100, i - 100, 0]]
        points += [[i - 100, 100, 0], [i - 100, -100, 0]]
        lines += [[i * 4, i * 4 + 1], [i * 4 + 2, i * 4 + 3]]
    colors = [GROUND_COLOR for i in range(len(lines))]
    ground_line = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    ground_line.colors = o3d.utility.Vector3dVector(colors)

    def update_point_cloud(vis):
        nonlocal rot_last

        # update targets
        if ptc_idx > 0:
            if show_recent:
                start_idx = max(0, ptc_idx - SHOW_RECENT_NUMBER)
            else:
                start_idx = 0
            end_idx = ptc_idx-len(frames[frame_idx][2])
            # show all frame
            if not show_now and end_idx:
                all_pcd.points = o3d.utility.Vector3dVector(
                    np.array(ptc[start_idx:end_idx]))
            else:
                all_pcd.points = o3d.utility.Vector3dVector(
                    np.array([[0, 0, 0]]))
            vis.update_geometry(all_pcd)
            # show now frame with purple color
            if len(frames[frame_idx][2]):
                now_pcd.points = o3d.utility.Vector3dVector(
                    np.array(ptc[end_idx:ptc_idx]))
                now_pcd.paint_uniform_color(NOW_FRAME_COLOR)
                vis.update_geometry(now_pcd)

        # update uav trace
        uav_trace.points = o3d.utility.Vector3dVector(
            np.array(trace[: frame_idx + 1]))
        uav_trace.paint_uniform_color(TRACE_COLOR)
        vis.update_geometry(uav_trace)

        # update uav position and pose
        position = frames[frame_idx][0]
        rot = R.from_quat(
            tuple(frames[frame_idx][1])).as_matrix()
        body_frame.rotate(rot @ rot_last.transpose())
        body_frame.translate(position, False)
        vis.update_geometry(body_frame)
        rot_last = rot

        print(frames[frame_idx][0], frames[frame_idx][3], frames[frame_idx][4])

    def key_forward_callback(vis):
        nonlocal frame_idx, ptc_idx
        if frame_idx < len(frames):
            ptc_idx += len(frames[frame_idx][2])
            update_point_cloud(vis)
            frame_idx += 1
        return True

    def key_backward_callback(vis):
        nonlocal frame_idx, ptc_idx
        if frame_idx > 0:
            frame_idx -= 1
            update_point_cloud(vis)
            ptc_idx -= len(frames[frame_idx][2])
        return True

    def key_show_recent_callback(vis):
        nonlocal show_recent
        show_recent = not show_recent
        update_point_cloud(vis)
        return True

    def key_show_now_callback(vis):
        nonlocal show_now
        show_now = not show_now
        update_point_cloud(vis)
        return True

    def key_show_ground_callback(vis):
        nonlocal show_ground
        show_ground = not show_ground
        if show_ground:
            vis.add_geometry(ground_line, False)
        else:
            vis.remove_geometry(ground_line, False)
        return True

    vis.create_window()
    vis.get_render_option().point_size = 1
    vis.get_render_option().background_color = np.asanyarray([0, 0, 0])

    for f in frames:
        ptc += f[2]
        trace.append(f[0])
    all_pcd.points = o3d.utility.Vector3dVector(np.array(ptc))
    vis.add_geometry(all_pcd)
    vis.add_geometry(now_pcd)

    uav_trace.points = o3d.utility.Vector3dVector(np.array(trace))
    uav_trace.paint_uniform_color(TRACE_COLOR)
    vis.add_geometry(uav_trace)

    enu_frame = o3d.geometry.TriangleMesh.create_coordinate_frame()
    vis.add_geometry(enu_frame)
    vis.add_geometry(body_frame)

    vis.add_geometry(ground_line)

    vis.register_key_callback(ord("D"), key_forward_callback)
    vis.register_key_callback(ord("A"), key_backward_callback)
    vis.register_key_callback(ord("R"), key_show_recent_callback)
    vis.register_key_callback(ord("N"), key_show_now_callback)
    vis.register_key_callback(ord("G"), key_show_ground_callback)
    vis.run()


if __name__ == "__main__":
    if len(sys.argv) == 1:
        print(USAGE)
        exit(0)

    # Parse args
    log_file = sys.argv[-1]
    save_pcd = False
    if len(sys.argv) >= 3 and sys.argv[2] == "-s":
        save_pcd = True

    install_vertical = False
    if "-r" in sys.argv:
        install_vertical = True

    # Parse radar bag
    if log_file.find("bag") != -1:
        frames = parse_ros_bag(log_file, install_vertical)
    elif log_file.find("radar_ptc") != -1:
        ret = data_to_bag(log_file)
        exit(0)
    elif os.path.isfile(log_file) == False:
        print(log_file)
        data_path_to_bag(log_file)
        exit(0)
    else:
        print("failed to parse invalid file, the file name should be started with 'bag'.")
        exit(0)

    # Save as pcd file
    if save_pcd:
        ptc = []
        for f in frames:
            ptc += f[2]
        now = datetime.datetime.now()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(ptc))
        o3d.io.write_point_cloud("obstacle_{}.pcd".format(
            now.strftime("%y%m%d%H%M")), pcd)

    # Show point cloud
    draw_point_cloud(frames)
