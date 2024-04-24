# CSCI739_Group4_Project

## Installation for Linux (works on school machines)
1. You might want to update apt `apt update`
2. Install cmake `apt install cmake`
3. If you don't already have git, get it: `apt install git`
4. Git clone vcpkg `git clone https://github.com/microsoft/vcpkg.git`
5. In vcpkg directory `cd vcpkg`, run the file `./bootstrap-vcpkg.sh`
6. Integrate install: `vcpkg integrate install`
7. Install PCL with visualization module - `vcpkg install pcl[visualization]` (This can take 30 minutes or so)
8. Git clone the repo `git clone https://github.com/samsitGit/CSCI739_Group4_Project.git`
9. Go to the project `cd CSCI739_Group4_Project` then configure the project with CMake: `cmake`
10. You may want to go into your 3dMapUpdates.cpp to change file paths for your ego and a complete trace unless you are on Irfan's computer.
11. Compile the program to get an executable: `make`
12. Run the file `./3dMapUpdates`

## Using PCP.py (point cloud player) and PCV.py (point cloud viewer)
```
conda create --name open3d-env python=3.6 # Create a new conda environment named "open3d-env" with Python 3.8
conda activate open3d-env # Activate the environment
pip install open3d==0.15.1 # Install specific version of Open3D
python PCP.py # after you change your file paths
python PCV.py # after you change your file paths

```