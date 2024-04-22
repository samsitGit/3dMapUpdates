# CSCI739_Group4_Project

## Installation for Windows
1. Follow this to set up and install CMake and vcpkg:
https://learn.microsoft.com/en-us/vcpkg/get_started/get-started-vs?pivots=shell-cmd
2. In vcpkg directory, run either bootstrap-vcpkg sh file or bat file.
3. Integrate install: `vcpkg integrate install`
4. Install PCL with vcpkg - `vcpkg install pcl:x64-windows-static`
5. Let your "directory" be the vcpkg.cmake located in scripts > buildsystems folder where you installed vcpkg.
6. Configure the project with CMake: `cmake -DVCPKG_TARGET_TRIPLEET=x64-windows-static -DCMAKE_TOOLCHAIN_FILE=<directory>`

## Using visualize.py
```
conda create --name open3d-env python=3.6 # Create a new conda environment named "open3d-env" with Python 3.8
conda activate open3d-env # Activate the environment
pip install open3d==0.15.1 # Install specific version of Open3D
python visualize.py # after you change your paths

```