# CSCI739_Group4_Project

## Installation
1. Follow this to set up and install CMake and vcpkg:
https://learn.microsoft.com/en-us/vcpkg/get_started/get-started-vs?pivots=shell-cmd

2. Run this command to set up vcpkg related libraries:
`vcpkg install`

3. Install PCL with vcpkg - `vcpkg install pcl`

4. Add vcpkg to your PATH in environmental variables.

5. Add vcpkg as VCPKG_ROOT to your environmental variables.

5. Modify your `CMakePresets.json` to set all `"DCMAKE_TOOLCHAIN_FILE"` (there are multiple) to vcpkg.cmake located in scripts > buildsystems folder where you installed vcpkg.

6. Now you can open this diretory in Visual Studio (install it if you haven't), and it will configure it.

## Using visualize.py

```
conda create --name open3d-env python=3.6 # Create a new conda environment named "open3d-env" with Python 3.8
conda activate open3d-env # Activate the environment
conda install -c open3d-admin open3d # Install Open3D from the open3d-admin channel
pip install open3d==0.15.1 # Install specific version of Open3D
python3 visualize.py # after you change your paths

```
Or run `pip install open3d`, will need Python 3.11. If you have a launcher, you can swap to it or use a Conda environment. If you already have it installed, you could also run it in PowerShell like so: `& "C:\Users\<your username>\AppData\Local\Programs\Python\Python311\python.exe" -m pip install open3d`

May want to update your environment with our root: `conda env update --name open3d-env --file environment_root.yml`
You do need to change the username of the prefix to your username

Finally, if it still doesn't work, just do `& "C:\Users\<your username>\AppData\Local\Programs\Python\Python311\python.exe" visualize.py`


