import open3d as o3d
import time
import os
import numpy as np
import re
from concurrent.futures import ThreadPoolExecutor

class DataPath:
    def __init__(self, directories):
        self.directories = [dir for dir in directories if os.path.exists(dir)]
        if not self.directories:
            raise FileNotFoundError("None of the specified directories exist.")

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
    def __init__(self, data_path, colors, camera_positions, lookat, up):
        self.data_path = data_path
        self.colors = colors
        self.camera_positions = camera_positions
        self.lookat = lookat
        self.up = up
        self.is_paused = True
        self.current_frame = 0
        self.pcds = []
        self.files_per_dir = [sorted(os.listdir(dir), key=extract_number) for dir in data_path.directories]
        self.max_frames = max(len(files) for files in self.files_per_dir) if self.files_per_dir else 0
        self.loaded_frames = 0
        self.load_frames(3)  # Initial load of 3 frames

    def load_frames(self, count):
        for dir_index, directory in enumerate(self.data_path.directories):
            start_index = self.loaded_frames
            end_index = start_index + count
            files_to_load = self.files_per_dir[dir_index][start_index:end_index]
            color_pair = self.colors[dir_index] if dir_index < len(self.colors) else ([1, 1, 1], [1, 1, 1])

            with ThreadPoolExecutor(max_workers=6) as executor:
                futures = [
                    executor.submit(read_and_color_pcd, os.path.join(directory, file), *color_pair)
                    for file in files_to_load
                ]
                new_pcds = [future.result() for future in futures]

            for idx, new_pcd in enumerate(new_pcds):
                frame_index = start_index + idx
                if frame_index < len(self.pcds):
                    self.pcds[frame_index] += new_pcd
                else:
                    self.pcds.append(new_pcd)

        self.loaded_frames += count

    def update_view(self, vis):
        if self.current_frame < len(self.pcds):
            vis.clear_geometries()
            vis.add_geometry(self.pcds[self.current_frame], reset_bounding_box=False)

    def run(self):
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window()
        vis.add_geometry(self.pcds[0])

        vis.register_key_callback(ord(","), lambda vis: self.step_backward(vis))
        vis.register_key_callback(ord("."), lambda vis: self.step_forward(vis))

        while True:
            vis.poll_events()
            vis.update_renderer()
            time.sleep(0.05)  # Reduce CPU usage

        vis.destroy_window()

    def step_forward(self, vis):
        if self.current_frame + 1 >= len(self.pcds) and self.current_frame + 1 < self.max_frames:
            self.load_frames(1)  # Load more frames dynamically
        self.current_frame = min(self.current_frame + 1, len(self.pcds) - 1)
        self.update_view(vis)

    def step_backward(self, vis):
        self.current_frame = max(self.current_frame - 1, 0)
        self.update_view(vis)
        
# Example usage:
ego1_path = "0"
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
vis = InteractiveVisualizer(data_path, colors, camera_positions, lookat, up)
vis.run()