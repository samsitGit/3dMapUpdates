# 3D Map Update System with LIDAR Trace and HD Map Integration

This project enables 3D map updates using LIDAR traces and a high-definition (HD) map of an environment. The system is divided into three main components:

1. **Visualizer** - Visualizes LIDAR trace and map updates.
2. **Client-Server Model** - Facilitates storage, updates, and downloading of the base map.
3. **CMake Project** - Processes LIDAR trace data to detect changes using a custom ray-tracing algorithm.

## Project Structure

- `visualizer.py`: Visualizes the LIDAR data and map updates.
- `map_cloud_server.py` & `map_cloud_client.py`: Components of the client-server model for base map updates.
- CMake project for ray-tracing algorithm-based trace processing.

## Setup

### Visualizer Setup

Create and activate the environment:

```bash
conda create --name open3d-env python=3.6 # Create the environment
conda activate open3d-env # Activate the environment
pip install open3d==0.15.1 # Install Open3D
```

After configuring your file paths, run:

```bash
python visualizer.py
```

You can use , . to go back or forward one frame respectively, depending on your step size. You can also pause, move around, ctrl+click, right click, and press R to reverse the player.

### Client-Server Model Setup

The client-server model allows storing, updating, and downloading changes to the base map.

```bash
python map_cloud_server.py # Run the server
python map_cloud_client.py # Run the client after setting the input trace path
```

### CMake Project Setup

The CMake project utilizes a previously captured LIDAR trace for detecting map updates. It features a custom ray-tracing algorithm for efficient change detection.

1. Ensure CMake is installed on your system.
2. Configure the project, build and run:

```bash
cmake 
make
./3dMapUpdates
```

3. Run the executable to process the LIDAR trace.

---

## Contributions

Feel free to contribute to the project by forking the repository, creating new branches, and submitting pull requests!

## Installation for Linux (works on school machines)
1. You might want to update apt `apt update`
2. Install cmake `apt install cmake`
3. If you don't already have git, get it: `apt install git`
4. Git clone vcpkg `git clone https://github.com/microsoft/vcpkg.git`
5. In vcpkg directory `cd vcpkg`, run the file `./bootstrap-vcpkg.sh`
6. Integrate install: `vcpkg integrate install`
7. Install PCL with visualization module - `vcpkg install pcl[visualization]` (This can take 30 minutes or so)
8. Git clone the repo `git clone https://github.com/samsitGit/3dMapUpdates`
9. Go to the project `cd 3dMapUpdates` then configure the project with CMake: `cmake`
10. You may want to go into your 3dMapUpdates.cpp to change file paths for your ego and a complete trace unless you are on Irfan's computer.
11. Compile the program to get an executable: `make`
12. Run the file `./3dMapUpdates`

## Installation for Windows
1. Follow this to set up and install CMake and vcpkg:
https://learn.microsoft.com/en-us/vcpkg/get_started/get-started-vs?pivots=shell-cmd
2. Run this command to set up vcpkg related libraries:
`vcpkg install`
3. Install PCL with vcpkg - `vcpkg install pcl`
4. Add vcpkg to your PATH in environmental variables.
5. Add vcpkg as VCPKG_ROOT to your environmental variables.
5. Modify your `CMakePresets.json` to set all `"DCMAKE_TOOLCHAIN_FILE"` (there are multiple) to vcpkg.cmake located in scripts > buildsystems folder where you installed vcpkg.
6. Now you can open this diretory in Visual Studio (install it if you haven't), and it will configure it.


