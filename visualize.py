import open3d as o3d
import time
import os
import numpy as np
import re

class DataPath:
    def __init__(self, ego_dir, base_map_path):
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
    center = pcd.get_center()
    R = pcd.get_rotation_matrix_from_xyz((0, 0, np.pi/2))
    pcd.rotate(R, center=center)
    return pcd

class InteractiveVisualizer:
    def __init__(self, data_path, camera_positions, lookat, up):
        self.data_path = data_path
        self.camera_positions = camera_positions
        self.lookat = lookat
        self.up = up
        self.is_paused = False
        self.current_frame = 0
        self.load_files()
        self.frame_count = len(self.pcds)

    def load_files(self):
        self.input_files = sorted(
            os.listdir(self.data_path.ego_dir),
            key=extract_number
        )
        self.pcds = [
            o3d.io.read_point_cloud(os.path.join(self.data_path.ego_dir, f))
            for f in self.input_files
        ]

    def toggle_pause(self, vis):
        self.is_paused = not self.is_paused
        print("Paused" if self.is_paused else "Running")

    def run(self):
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window()
        vis.add_geometry(self.pcds[0])

        vis.register_key_callback(ord(" "), self.toggle_pause)
        
        start_time = time.time()
        frames_displayed = 0

        while self.current_frame < self.frame_count:
            vis.poll_events()
            vis.update_renderer()
            if not self.is_paused:
                vis.remove_geometry(self.pcds[self.current_frame], reset_bounding_box=False)
                self.current_frame += 1
                if self.current_frame < self.frame_count:
                    vis.add_geometry(self.pcds[self.current_frame], reset_bounding_box=False)
                current_time = time.time()
                frames_displayed += 1
                fps = frames_displayed / (current_time - start_time)
                print(f"FPS: {fps:.2f}\tProgress: {100 * self.current_frame / self.frame_count:.2f}%", end='\r')
                time.sleep(0.005) 
            if self.current_frame >= self.frame_count:
                break

        vis.destroy_window()

# Example usage:
ego_dir="G:/ai proj dl/project/project/map_compression/data_1/ego/town02/w_dropoff/w_noise/vehicles=6/day1/pcds"
basemap="G:/ai proj dl/project/project/map_compression/data_1/basemaps/town02/w_dropoff/w_noise/pcds"
data_path = DataPath(ego_dir, basemap)
camera_positions = [[1, 1, -1], [20, 20, 20]] 
lookat = [0, 0, 0]
up = [0, 1, 0]
vis = InteractiveVisualizer(data_path, camera_positions, lookat, up)
vis.run()
