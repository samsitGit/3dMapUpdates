import sys
import open3d as o3d
import time
import os
import numpy as np
import re
from concurrent.futures import ThreadPoolExecutor

class DataPath:
    def __init__(self, static_files, directories):
        self.static_files = [file for file in static_files if os.path.exists(file)]
        if not self.static_files:
            print("No additional static files available.")
        self.directories = [dir for dir in directories if os.path.exists(dir)]
        if not self.directories:
            print("No additional directories available.")

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
    def __init__(self, data_path, sta_colors, ego_colors, step_size, camera_positions, lookat, up, video):
        self.data_path = data_path
        self.ego_colors = ego_colors
        self.sta_colors = sta_colors
        self.step_size = step_size
        self.camera_positions = camera_positions
        self.lookat = lookat
        self.up = up
        self.current_frame = 0
        self.load_static_files()
        self.video = video

        if video:
            self.is_paused = False
            self.reverse_playback = False
            self.load_files()
            self.frame_count = len(self.pcds)

        if not video:
            self.is_paused = True
            self.pcds = []
            self.files_per_dir = [sorted(os.listdir(dir), key=extract_number) for dir in data_path.directories]
            self.max_frames = max(len(files) for files in self.files_per_dir) if self.files_per_dir else 0
            self.loaded_frames = 0
            self.load_frames(3)  # Initial load of 3 frames

    def load_static_files(self):
        self.static_pcds = []
        for file, color in zip(self.data_path.static_files, self.sta_colors):
            self.static_pcds.append(read_and_color_pcd(file, color[0], color[1]))

    def load_files(self):
        self.pcds = []
        for i, directory in enumerate(self.data_path.directories):
            input_files = sorted(
                os.listdir(directory),
                key=extract_number
            )
            color_pair = self.ego_colors[i] if i < len(self.ego_colors) else (RED, RED)  # Default to red if no color specified
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

    def load_frames(self, count):
        for dir_index, directory in enumerate(self.data_path.directories):
            start_index = self.loaded_frames
            end_index = start_index + count
            files_to_load = self.files_per_dir[dir_index][start_index:end_index]
            color_pair = self.ego_colors[dir_index] if dir_index < len(self.ego_colors) else ([1, 1, 1], [1, 1, 1])

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

    def toggle_pause(self, vis):
        self.is_paused = not self.is_paused
        print(f"\nPaused" if self.is_paused else "Running")

    def update_view(self, vis):
        vis.clear_geometries()
        for pcd in self.static_pcds:
            vis.add_geometry(pcd, reset_bounding_box=False)
        if self.current_frame < len(self.pcds):
            vis.add_geometry(self.pcds[self.current_frame], reset_bounding_box=False)

    def step_forward(self, vis):
        print(f"You're on frame {self.current_frame}.")
        if self.video:
            if not self.is_paused:
                self.toggle_pause(vis)
            self.current_frame = min(self.current_frame + self.step_size, self.frame_count - 1)

        if not self.video:
            if self.current_frame + 1 >= len(self.pcds) and self.current_frame + 1 < self.max_frames:
                self.load_frames(1)  # Load more frames dynamically
            self.current_frame = min(self.current_frame + 1, len(self.pcds) - 1)
        self.update_view(vis)

    def step_backward(self, vis):
        print(f"You're on frame {self.current_frame}.")
        if self.video:
            if not self.is_paused:
                self.toggle_pause(vis)
            self.current_frame = max(self.current_frame - self.step_size, 0)
        if not self.video:
            self.current_frame = max(self.current_frame - 1, 0)
        self.update_view(vis)

    def toggle_reverse(self, vis):
        self.reverse_playback = not self.reverse_playback
        if self.is_paused:
            self.is_paused = not self.is_paused
        print("\nReversed" if self.reverse_playback else "Normal Playback")

    def run(self):
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window()
        for pcd in self.static_pcds:
            vis.add_geometry(pcd)
        if self.pcds:
            vis.add_geometry(self.pcds[0])

        vis.register_key_callback(ord(","), lambda vis: self.step_backward(vis))
        vis.register_key_callback(ord("."), lambda vis: self.step_forward(vis))
        if self.video:
            vis.register_key_callback(ord(" "), self.toggle_pause)
            vis.register_key_callback(ord("R"), self.toggle_reverse)
            start_time = time.time()
            frames_displayed = 0

        while True:
            vis.poll_events()
            vis.update_renderer()
            
            if self.video:
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
sta1_path = "aa699appeared.pcd"
sta2_path = ""
sta3_path = "no static"
sta4_path = "no static"
sta5_path = "no static"
sta6_path = "no static"
ego1_path = "ego_localized"
ego2_path = "2"
ego3_path = "no ego"
ego4_path = "no ego"
ego5_path = "no ego"
ego6_path = "no ego"

static_files = [sta1_path, sta2_path, sta3_path, sta4_path, sta5_path, sta6_path]
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
sta1_color = (RED, RED)
sta2_color = (RED, RED)
sta3_color = (RED, RED)
sta4_color = (RED, RED)
sta5_color = (RED, RED)
sta6_color = (RED, RED)

ego1_color = (LIGHT_BLUE, BLUE)
ego2_color = (RED, PALE_RED)
ego3_color = (BLUE, LIGHT_BLUE)
ego4_color = (YELLOW, PALE_RED)
ego5_color = (CYAN, LIGHT_CYAN)
ego6_color = (MAGENTA, LIGHT_MAGENTA)

sta_colors = [sta1_color, sta2_color, sta3_color,
          sta4_color, sta5_color, sta6_color]

ego_colors = [ego1_color, ego2_color, ego3_color,
          ego4_color, ego5_color, ego6_color]

mode = sys.argv[1] # 'video' or anything
video_player = mode == 'video'
data_path = DataPath(static_files, directories)
camera_positions = [[1, 1, -1], [20, 20, 20]]
lookat = [0, 0, 0]
up = [0, 1, 0]
step_size = 2 # you can change step size of the frames here. keep in mind frame mode is 1 step size for proper management
vis = InteractiveVisualizer(data_path, sta_colors, ego_colors, step_size, camera_positions, lookat, up, video_player)
vis.run()