# Radar Vis - Show radar detections.
# Copyright 2008 TopXGun Robotics.  All rights reserved.
#
# Dai Yuan Yuan yydai@topxgun.com
# version :
#           0.3 支持MindCruise格式。
#           0.2 支持仿地雷达点云解析。
#           0.1 支持避障雷达点云解析。

import csv
import datetime
import sys
from math import sqrt

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

USAGE = "Parse and Show TopxGun Radar PointCloud Log.\n\
    Usage: python3 radar_vis.py radar_xxx.data [-s]\n\n\
    D - move forward a frame.\n\
    A - move backward a frame.\n\
    N - only show now frame.\n\
    R - only show recent frames."

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
INSTALL_VERTICAL_4D_ANGLE = 20 # 安装角度

def parse_mind_frame(file_path, vertical_install=False):
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
    with open(file_path, "r") as f:
        reader = csv.reader(f, delimiter=" ", skipinitialspace=True)
        for row in reader:
            if len(row) <= 14:
                continue

            targets = []
            time_str = row[1]
            coord_type = int(row[3])
            radar_type = int(row[4])
            version_num = int(row[2])
            position = np.array([float(row[5]), float(row[6]), float(row[7])])
            vel = np.array([float(row[9]), float(row[10]), float(row[11])])
            pose = np.array([float(row[12]), float(row[13]), float(row[14])])
            pose_rate = np.array([float(row[15]), float(row[16]), float(row[17])])
            install_angle = float(row[18])
            if (version_num == 200):
                segment_count = 12
            else:
                segment_count = 7

                
            if len(target_frames) > 5:
            #if False:
                position_ = target_frames[-2][1]
                pose_ = target_frames[-2][2]
            else:
                position_ = position
                pose_ = pose

            rot_ned_body = R.from_euler("xyz", tuple(pose_), True).as_matrix()
            if vertical_install:
                install_angle = -1*INSTALL_VERTICAL_4D_ANGLE
                rot_radar_install = R.from_euler("xyz", (90, install_angle * -1.0, 0), degrees=True).as_matrix()
            else:
                rot_radar_install = R.from_euler("y", install_angle * -1.0, degrees=True).as_matrix()
            rot_enu_radar = None
            if radar_type == RADAR_TYPE_FRONT:
                rot_enu_radar = rot_enu_ned @ rot_ned_body @ rot_body_fradar
            elif radar_type == RADAR_TYPE_REAR:
                rot_enu_radar = rot_enu_ned @ rot_ned_body @ rot_body_rradar
            else:
                continue

            for idx in range(19, len(row) - 1, segment_count):
                confidence = int(row[idx + 5])
                if abs(confidence) < CONFIDENCE_LIMIT:
                    continue

                if coord_type == 0:
                    d, azimuth, elevation = float(row[idx]), float(row[idx+1]), float(row[idx + 2])
                    rot_radar_polar = R.from_euler("xyz", (0, elevation, azimuth), True).as_matrix()
                    target = [d, 0, 0]
                    targets.append(rot_enu_radar @ (rot_radar_install @ rot_radar_polar @ np.array(target) + t_body_fradar)+ position_)
                elif coord_type == 1:
                    target = [float(row[idx]), float(row[idx+1]), float(row[idx + 2])]
                    targets.append(rot_enu_radar @ rot_radar_install @  np.array(target) + position_)

            if max([abs(rate) for rate in pose_rate[0:2]]) > POSE_RATE_LIMIT:
                targets = []

            if abs(pose_rate[2]) > YAW_RATE_LIMIT:
                targets = []

            target_frames.append([time_str, position, pose, targets, pose_rate, vel])

    return target_frames


def parse_obs_frame(file_path):
    # Log format: date time log_type radar_type uav_x uav_y uav_z terrian_height roll pitch yaw install_angle 0 [x y z vel rcs ...]
    # Log format: date time version radar_type uav_x uav_y uav_z vel_n vel_e vel_u roll pitch yaw roll_vel pitch_vel yaw_vel terrian_height mount_angle ptz_vel [ x y z vel rcs ... ]
    target_frames = []
    with open(file_path, "r") as f:
        reader = csv.reader(f, delimiter=" ", skipinitialspace=True)
        for row in reader:
            if len(row) < 3:
                continue

            time_str = row[1]
            version = row[2]
            radar_type = int(row[3])
            if version == "100":
                header_len = 19
                if len(row) <= header_len:
                    continue
                position = np.array([float(row[4]), float(row[5]), float(row[6])])
                pose = np.array([float(row[10]), float(row[11]), float(row[12])])
                mount_angle = float(row[17])
            else:
                header_len = 13
                if len(row) <= header_len:
                    continue
                position = np.array([float(row[4]), float(row[5]), float(row[6])])
                pose = np.array([float(row[8]), float(row[9]), float(row[10])])
                mount_angle = float(row[11])

            """ to_land = float(row[7])
            if to_land <= TO_LAND_LIMIT:
                continue """

            rot_ned_body = R.from_euler("xyz", tuple(pose), True).as_matrix()
            rot_radar_install = R.from_euler("y", mount_angle * -1.0, degrees=True).as_matrix()
            rot_enu_radar = None
            if radar_type == RADAR_TYPE_FRONT:
                rot_enu_radar = rot_enu_ned @ rot_ned_body @ rot_body_fradar @ rot_radar_install
            elif radar_type == RADAR_TYPE_REAR:
                rot_enu_radar = rot_enu_ned @ rot_ned_body @ rot_body_rradar @ rot_radar_install
            else:
                continue

            targets = [] 
            for idx in range(header_len, len(row) - 1, 5):
                target = [float(row[idx]), float(row[idx + 1]), float(row[idx + 2])]
                targets.append(rot_enu_radar @ np.array(target) + position)
            target_frames.append([time_str, position, pose, targets, [0, 0, 0]])

    return target_frames


def parse_terrain_frame(file_path):
    # Log format: date time frame_id lon lat uav_x uav_y uav_z roll pitch yaw vel_n vel_e vel_u [ x y z vel rcs ... ]
    # Log format: date time version radar_type uav_x uav_y uav_z vel_n vel_e vel_u roll pitch yaw roll_vel pitch_vel yaw_vel terrian_height mount_angle ptz_vel [ x y z vel rcs ... ]
    target_frames = []
    with open(file_path, "r") as f:
        reader = csv.reader(f, delimiter=" ", skipinitialspace=True)
        for row in reader:
            if len(row) < 3:
                continue

            time_str = row[1]
            version = row[2]
            if version == "100":
                header_len = 19
                if len(row) <= header_len:
                    continue
                position = np.array([float(row[4]), float(row[5]), float(row[6])])
                pose = np.array([float(row[10]), float(row[11]), float(row[12])])
                mount_angle = float(row[17])
            else:
                header_len = 14
                if len(row) <= header_len:
                    continue
                position = np.array([float(row[5]), float(row[6]), float(row[7])])
                pose = np.array([float(row[8]), float(row[9]), float(row[10])])
                mount_angle = 0

            rot_ned_body = R.from_euler("xyz", tuple(pose), True).as_matrix()
            rot_body_radar = R.from_euler("xyz", (0, mount_angle, 0), True).as_matrix()
            rot_enu_radar = rot_enu_ned @ rot_ned_body @ rot_body_radar

            targets = []
            for idx in range(header_len, len(row) - 1, 5):
                rcs = int(row[idx+4])
                if rcs < RCS_THRESHOLD:
                    continue
                target = [float(row[idx]), float(row[idx + 1]), float(row[idx + 2])]
                targets.append(rot_enu_radar @ np.array(target) + position)
            target_frames.append([time_str, position, pose, targets, [0, 0, 0]])

    return target_frames


def parse_terrain_single(file_path):
    # terrain single log format:
    # date time lat lon roll pitch yaw x y alt vel_n vel_e vel_u
    # terrain_raw terrain_raw_raw terrain_confidence terrain_signal_quality terrain_detect_angle terrain_optimized valid
    velocity = []
    height = []
    terrain = []
    to_terrain_raw = []
    to_terrain_raw_raw = []
    to_terrain = []
    valid_flag = []

    with open(file_path) as f:
        reader = csv.reader(f, delimiter=" ", skipinitialspace=True)
        try:
            for row in reader:
                if len(row) > 18:
                    velocity.append(sqrt(float(row[10])**2 + float(row[11])**2))
                    height.append(float(row[9]))
                    to_terrain_raw.append(float(row[15]))
                    to_terrain_raw_raw.append(float(row[16]))
                    to_terrain.append(float(row[18]))
                    valid_flag.append(int(row[19]))
                    if valid_flag[-1]:
                        terrain.append(height[-1]-to_terrain[-1])
                    else:
                        terrain.append(0)
        except csv.Error as e:
            pass

    plt.close('all')
    fig, ax1 = plt.subplots()

    ax1.plot(height, label="height")
    ax1.plot(to_terrain, label="to_terrain")
    ax1.plot(terrain, label="terrain")
    ax1.plot(valid_flag, label="terrain_valid")
    ax1.set_ylabel("height@m")
    ax1.set_xlabel("time@50ms")
    ax1.legend(loc='upper left')

    ax2 = ax1.twinx()
    ax2.plot(velocity, label="velocity", color="gray")
    ax2.set_ylabel("velocity@m/s")
    ax2.set_ylim(-30,30)
    ax2.legend(loc='upper right')

    plt.title("Terrain Height Analysis")
    plt.show()


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
            end_idx = ptc_idx-len(frames[frame_idx][3])
            # show all frame
            if not show_now and end_idx:
                all_pcd.points = o3d.utility.Vector3dVector(np.array(ptc[start_idx:end_idx]))
            else:
                all_pcd.points = o3d.utility.Vector3dVector(np.array([[0, 0, 0]]))
            vis.update_geometry(all_pcd)
            # show now frame with purple color
            if len(frames[frame_idx][3]):
                now_pcd.points = o3d.utility.Vector3dVector(np.array(ptc[end_idx:ptc_idx]))
                now_pcd.paint_uniform_color(NOW_FRAME_COLOR)
                vis.update_geometry(now_pcd)

        # update uav trace
        uav_trace.points = o3d.utility.Vector3dVector(np.array(trace[: frame_idx + 1]))
        uav_trace.paint_uniform_color(TRACE_COLOR)
        vis.update_geometry(uav_trace)

        # update uav position and pose
        position = frames[frame_idx][1]
        rot = (rot_enu_ned @ R.from_euler("xyz", tuple(frames[frame_idx][2]), True).as_matrix())
        body_frame.rotate(rot @ rot_last.transpose())
        body_frame.translate(position, False)
        vis.update_geometry(body_frame)
        rot_last = rot

        print(frames[frame_idx][0], frames[frame_idx][2], frames[frame_idx][4])

    def key_forward_callback(vis):
        nonlocal frame_idx, ptc_idx
        if frame_idx < len(frames):
            ptc_idx += len(frames[frame_idx][3])
            update_point_cloud(vis)
            frame_idx += 1
        return True

    def key_backward_callback(vis):
        nonlocal frame_idx, ptc_idx
        if frame_idx > 0:
            frame_idx -= 1
            update_point_cloud(vis)
            ptc_idx -= len(frames[frame_idx][3])
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
        ptc += f[3]
        trace.append(f[1])
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

    # Parse radar log
    if log_file.find("radar_ptc") != -1:
        frames = parse_mind_frame(log_file, install_vertical)
    elif log_file.find("tcas_ptc") != -1:
        frames = parse_obs_frame(log_file)
    elif log_file.find("terrain_ptc") != -1:
        frames = parse_terrain_frame(log_file)
    elif log_file.find("terrain_single") != -1:
        parse_terrain_single(log_file)
        exit(0)
    else:
        print("failed to parse invalid file, the file name should be started with 'tcas_ptc' or 'terrain_ptc'")
        exit(0)

    # Save as pcd file
    if save_pcd:
        ptc = []
        for f in frames:
            ptc += f[3]
        now = datetime.datetime.now()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(ptc))
        o3d.io.write_point_cloud("obstacle_{}.pcd".format(now.strftime("%y%m%d%H%M")), pcd)

    # Show point cloud
    draw_point_cloud(frames)
