import open3d as o3d
from time import sleep,time
import os
import numpy as np
import re
import sys

class DataPath:
    def __init__(self, ego_dir,base_map_path):
        if not os.path.exists(ego_dir):
            raise FileNotFoundError(f"{ego_dir} does not exist")
        if not os.path.exists(base_map_path):
            raise FileNotFoundError(f"{base_map_path} does not exist")
        self.ego_dir = ego_dir
        self.base_map_dir = base_map_path

def extract_number(filename):
    match = re.search(r'\d+', filename)
    return int(match.group()) if match else float('inf')

def rotate_point_cloud(pcd):
    center=pcd.get_center()
    R = pcd.get_rotation_matrix_from_xyz((0, 0, 90))
    pcd.rotate(R, center=center)
    return pcd

def show_video(dataPath: DataPath, camera_positions=None, lookat=None, up=None):
    input_dir = dataPath.ego_dir
    input_files = os.listdir(input_dir)
    frame_count = len(input_files)
    input_files = sorted(input_files, key=extract_number)
    # read first pcd file
    pcd = o3d.io.read_point_cloud(os.path.join(input_dir, input_files[0]))

    # getting open3d to display the video
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    frames_displayed=0

    ctr = vis.get_view_control()
    ctr.set_lookat(lookat)
    ctr.set_up(up)
    ctr.set_front(camera_positions[0])

    frames_displayed = 0
    start = time()

    # iterate through remaining files
    for idx, input_file in enumerate(input_files[1:], start=1):
        vis.remove_geometry(pcd, False)
        pcd = o3d.io.read_point_cloud(os.path.join(input_dir, input_file))
        vis.add_geometry(pcd)

        if idx < len(camera_positions):
            ctr.set_front(camera_positions[idx % len(camera_positions)])

        vis.poll_events()
        vis.update_renderer()

        frames_displayed+=1
        current_time=time()
        #FPS isnt precise, but it gives a good estimate. Could be improved by using a sliding window of number of frames displayed in the last second 
        print(f"FPS: {frames_displayed/(current_time-start):.7f}",f"\t Progress: {frames_displayed/frame_count*100:.7f}%",end="\r") 
        sleep(0.005) # makes display a little smoother (display will not match fps of video)

    vis.destroy_window()

def show_pcd(dataPath:DataPath,frame_number):
    input_dir = dataPath.ego_dir
    pcd = o3d.io.read_point_cloud(os.path.join(input_dir, f"{frame_number}.pcd"))
    print(pcd.points)
    #o3d.visualization.draw_geometries([pcd])

def show_pcd_file(path):
    pcd = o3d.io.read_point_cloud(path)
    o3d.visualization.draw_geometries([pcd])

ego_dir="G:/ai proj dl/project/project/map_compression/data_1/ego/town02/w_dropoff/w_noise/vehicles=6/day1/pcds"
basemap="G:/ai proj dl/project/project/map_compression/data_1/basemaps/town02/w_dropoff/w_noise/pcds"
dataPath=DataPath(ego_dir,basemap)
# params
camera_positions = [[0,0,0], [20, 20, 20]] #xyz
lookat = [0, 0, 0]
up = [0, 1, 0]
option = int(sys.argv[1]) 
if option == 1:
    show_video(dataPath, camera_positions=camera_positions, lookat=lookat, up=up)
    print(1)
    print()


#show_video(dataPath)
# show_pcd_file("C:/Users/irfan/OneDrive/Desktop/courses/topics in ai/project/src/3dMapUpdates/out/build/x64-release/clipped_cloud.pcd")