import open3d as o3d
import time
import os
import numpy as np
import re
from concurrent.futures import ThreadPoolExecutor

class DataPath:
    def __init__(self, directories):
        self.directories = []
        for dir in directories:
            if os.path.exists(dir):
                self.directories.append(dir)
            else:
                print(f"Warning: {dir} does not exist and will be skipped.")

def extract_number(filename):
    match = re.search(r'\d+', filename)
    return int(match.group()) if match else None

def read_and_color_pcd(filepath, start_color=None, end_color=None):
    pcd = o3d.io.read_point_cloud(filepath)
    if start_color and end_color:
        points = np.asarray(pcd.points)
        colors = np.linspace(np.array(start_color), np.array(end_color), len(points))
        pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd

class InteractiveVisualizer:
    def __init__(self, data_path, colors, step_size, camera_positions, lookat, up):
        self.data_path = data_path
        self.colors = colors
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
        self.pcds = []
        for i, directory in enumerate(self.data_path.directories):
            input_files = sorted(
                os.listdir(directory),
                key=extract_number
            )
            color_pair = self.colors[i] if i < len(self.colors) else ([1, 1, 1], [1, 1, 1])  # Default to white if no color specified
            with ThreadPoolExecutor(max_workers=6) as executor:
                futures = [
                    executor.submit(
                        read_and_color_pcd,
                        os.path.join(directory, filename),
                        color_pair[0], color_pair[1]
                    ) for filename in input_files
                ]
                pcds_for_dir = [future.result() for future in futures]
            if not self.pcds:
                self.pcds = pcds_for_dir
            else:
                # Ensure lengths match for combining, fill with empty PCDs if necessary
                max_len = max(len(self.pcds), len(pcds_for_dir))
                self.pcds.extend([o3d.geometry.PointCloud()] * (max_len - len(self.pcds)))
                pcds_for_dir.extend([o3d.geometry.PointCloud()] * (max_len - len(pcds_for_dir)))
                self.pcds = [pcd1 + pcd2 for pcd1, pcd2 in zip(self.pcds, pcds_for_dir)]

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
                time.sleep(0.025) 
            if self.current_frame == 0 or self.current_frame == self.frame_count - 1:
                break

        vis.destroy_window()

# Example usage:
ego1_path = "clusteredappearedpoints"
ego2_path = "1"
ego3_path = "ego3"
ego4_path = "ego4"
ego5_path = "ego5"
ego6_path = "ego6"
directories = [ego1_path, ego2_path, ego3_path, ego4_path, ego5_path, ego6_path]

# Define color constants
RED = [1, 0, 0]
ORANGE = [1, 0.5, 0]
GREEN = [0, 1, 0]
LIGHT_GREEN = [0.5, 1, 0]
BLUE = [0, 0, 1]
LIGHT_BLUE = [0, 0.5, 1]
YELLOW = [1, 1, 0]
PALE_RED = [1, 0.5, 0.5]
CYAN = [0, 1, 1]
LIGHT_CYAN = [0.5, 1, 1]
MAGENTA = [1, 0, 1]
LIGHT_MAGENTA = [1, 0.5, 1]

# Define gradient variables
# to have one constant color, just do (RED, RED) for example
# pass in None instead if you want default colors
ego1_color = (RED, RED)
ego2_color = (LIGHT_BLUE, BLUE)
ego3_color = (BLUE, LIGHT_BLUE)
ego4_color = (YELLOW, PALE_RED)
ego5_color = (CYAN, LIGHT_CYAN)
ego6_color = (MAGENTA, LIGHT_MAGENTA)

colors = [ego1_color, ego2_color, ego3_color,
          ego4_color, ego5_color, ego6_color]
data_path = DataPath(directories)
camera_positions = [[1, 1, -1], [20, 20, 20]]
lookat = [0, 0, 0]
up = [0, 1, 0]
step_size = 2 # you can change step size of the frames here
vis = InteractiveVisualizer(data_path, colors, step_size, camera_positions, lookat, up)
vis.run()
