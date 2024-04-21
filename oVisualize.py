import open3d as o3d
import time
import os
import numpy as np
import re

class DataPath:
    def __init__(self, ego_dir, base_map_path, ego2_dir):
        if not os.path.exists(ego_dir):
            raise FileNotFoundError(f"{ego_dir} does not exist")
        if not os.path.exists(base_map_path):
            raise FileNotFoundError(f"{base_map_path} does not exist")
        if not os.path.exists(ego2_dir):
            raise FileNotFoundError(f"{ego2_dir} does not exist")
        self.ego_dir = ego_dir
        self.base_map_dir = base_map_path
        self.ego2_dir = ego2_dir

def extract_number(filename):
    match = re.search(r'\d+', filename)
    return int(match.group()) if match else None

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
        input_files_ego = sorted(
            os.listdir(self.data_path.ego_dir),
            key=extract_number
        )
        input_files_ego2 = sorted(
            os.listdir(self.data_path.ego2_dir),
            key=extract_number
        )
        self.pcds = []
        max_len = max(len(input_files_ego), len(input_files_ego2))
        for i in range(max_len):
            combined_pcd = o3d.geometry.PointCloud()
            if i < len(input_files_ego):
                pcd = o3d.io.read_point_cloud(os.path.join(self.data_path.ego_dir, input_files_ego[i]))
                pcd.paint_uniform_color([1, 0, 0])  # Red color
                combined_pcd += pcd
            if i < len(input_files_ego2):
                pcd = o3d.io.read_point_cloud(os.path.join(self.data_path.ego2_dir, input_files_ego2[i]))
                pcd.paint_uniform_color([0, 1, 0])  # Green color
                combined_pcd += pcd
            self.pcds.append(combined_pcd)

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
ego_dir = "ego"
basemap_dir = "basemap"
ego2_dir = "ego2"
data_path = DataPath(ego_dir, basemap_dir, ego2_dir)
camera_positions = [[1, 1, -1], [20, 20, 20]] 
lookat = [0, 0, 0]
up = [0, 1, 0]
vis = InteractiveVisualizer(data_path, camera_positions, lookat, up)
vis.run()
