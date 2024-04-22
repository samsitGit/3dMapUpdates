import open3d as o3d
import time
import os
import numpy as np
import re
from concurrent.futures import ThreadPoolExecutor

class DataPath:
    def __init__(self, ego_dir, ego2_dir=None):
        if not os.path.exists(ego_dir):
            raise FileNotFoundError(f"{ego_dir} does not exist")
        self.ego_dir = ego_dir
        self.ego2_dir = ego2_dir if ego2_dir and os.path.exists(ego2_dir) else None

def extract_number(filename):
    match = re.search(r'\d+', filename)
    return int(match.group()) if match else None

def read_and_color_pcd(filepath, start_color, end_color):
    pcd = o3d.io.read_point_cloud(filepath)
    points = np.asarray(pcd.points)
    colors = np.linspace(start_color, end_color, len(points))
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd

class InteractiveVisualizer:
    def __init__(self, data_path, step_size, camera_positions, lookat, up):
        self.data_path = data_path
        self.step_size = step_size
        self.camera_positions = camera_positions
        self.lookat = lookat
        self.up = up
        self.is_paused = False
        self.reverse_playback = False
        self.current_frame = 0
        self.load_files()
        self.frame_count = len(self.pcds)

    def load_files(self):
        input_files_ego = sorted(
            os.listdir(self.data_path.ego_dir),
            key=extract_number
        )
        
        with ThreadPoolExecutor(max_workers=6) as executor:
            futures_ego = [
                executor.submit(
                    read_and_color_pcd, 
                    os.path.join(self.data_path.ego_dir, filename), 
                    [0, 0, 1], [1, 0, 0]  # Blue to red gradient
                ) for filename in input_files_ego
            ]

        self.pcds = [future.result() for future in futures_ego]

        if self.data_path.ego2_dir:
            input_files_ego2 = sorted(
                os.listdir(self.data_path.ego2_dir),
                key=extract_number
            )
            with ThreadPoolExecutor(max_workers=6) as executor:
                futures_ego2 = [
                    executor.submit(
                        read_and_color_pcd, 
                        os.path.join(self.data_path.ego2_dir, filename), 
                        [0, 1, 0], [1, 1, 0]  # Green to yellow gradient
                    ) for filename in input_files_ego2
                ]
            
            for i, future in enumerate(futures_ego2):
                self.pcds[i] += future.result()  # Combine with corresponding PCDs from the first directory

    def toggle_pause(self, vis):
        self.is_paused = not self.is_paused
        print("Paused" if self.is_paused else "Running")

    def step_forward(self, vis):
        if not self.is_paused:
            self.toggle_pause(vis)
        self.current_frame = min(self.current_frame + self.step_size, self.frame_count - 1)
        self.update_view(vis)

    def step_backward(self, vis):
        if not self.is_paused:
            self.toggle_pause(vis)
        self.current_frame = max(self.current_frame - self.step_size, 0)
        self.update_view(vis)

    def toggle_reverse(self, vis):
        self.reverse_playback = not self.reverse_playback
        print("Reversed" if self.reverse_playback else "Normal Playback")

    def update_view(self, vis):
        vis.clear_geometries()
        vis.add_geometry(self.pcds[self.current_frame], reset_bounding_box=False)

    def run(self):
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window()
        vis.add_geometry(self.pcds[0])

        vis.register_key_callback(ord(" "), self.toggle_pause)
        vis.register_key_callback(ord(","), self.step_backward)
        vis.register_key_callback(ord("."), self.step_forward)
        vis.register_key_callback(ord("R"), self.toggle_reverse)

        start_time = time.time()
        frames_displayed = 0

        while True:
            vis.poll_events()
            vis.update_renderer()
            if not self.is_paused:
                if self.reverse_playback:
                    self.current_frame = max(self.current_frame - self.step_size, 0)
                else:
                    self.current_frame = min(self.current_frame + self.step_size, self.frame_count - 1)
                
                self.update_view(vis)
                
                current_time = time.time()
                frames_displayed += 1
                fps = frames_displayed / (current_time - start_time)
                print(f"FPS: {fps:.2f}\tProgress: {100 * self.current_frame / self.frame_count:.2f}%", end='\r')
                time.sleep(0.005) 
            if self.current_frame == 0 or self.current_frame == self.frame_count - 1:
                break

        vis.destroy_window()

# Example usage:
ego_dir = "ego"
ego2_dir = "ego2"
data_path = DataPath(ego_dir, ego2_dir)
camera_positions = [[1, 1, -1], [20, 20, 20]] 
lookat = [0, 0, 0]
up = [0, 1, 0]
step_size = 5
vis = InteractiveVisualizer(data_path, step_size, camera_positions, lookat, up)
vis.run()
