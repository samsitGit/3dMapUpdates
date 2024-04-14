import open3d as o3d
from time import sleep,time
import os
import numpy as np
import re

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

def show_video(dataPath:DataPath):
    input_dir = dataPath.ego_dir
    input_files = os.listdir(input_dir)
    frame_count=len(input_files)
    input_files = sorted(input_files, key=extract_number)
    # read first pcd file
    pcd = o3d.io.read_point_cloud(os.path.join(input_dir, input_files[0]))

    # getting open3d to display the video
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    frames_displayed=0
    start=time()
    # iterate through remaining files     
    for input_file in input_files[1:]:
        input_file = os.path.join(input_dir, input_file)
        # remove previous pointcloud and add new one    
        vis.remove_geometry(pcd,False)
        pcd = o3d.io.read_point_cloud(input_file) # Read the point cloud
        rotated_pcd = rotate_point_cloud(pcd)
        
        # add pcd to visualizer
        vis.add_geometry(rotated_pcd)
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
show_video(dataPath)
# show_pcd_file("C:/Users/irfan/OneDrive/Desktop/courses/topics in ai/project/src/3dMapUpdates/out/build/x64-release/clipped_cloud.pcd")