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